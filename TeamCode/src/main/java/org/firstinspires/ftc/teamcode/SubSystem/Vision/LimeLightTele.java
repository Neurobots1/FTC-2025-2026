package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.control.LowPassFilter;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
@TeleOp(name = "LimeLightTele", group = "Test")
public class LimeLightTele extends OpMode {

    private Limelight3A limelight;
    private LLResult llResult;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime resetCooldown = new ElapsedTime();

    private KalmanFilter kalX;
    private KalmanFilter kalY;
    private LowPassFilter headingErrLP;

    public boolean filtersInit = false;

    // Pedro estimates (inches + radians)
    public double estX = 72.0;
    public double estY = 72.0;
    public double estH = Math.toRadians(90.0);

    public boolean lastA = false;
    public boolean lastB = false;

    public static final double RESET_COOLDOWN_S = 0.40;
    public static final double HEADING_LP_ALPHA = 0.15;

    public static final double KAL_MODEL_COV = 0.05;
    public static final double KAL_DATA_COV  = 2.0;

    public static final double MAX_STEP_IN  = 18.0;
    public static final double MAX_STEP_RAD = Math.toRadians(30.0);

    public static final int PIPELINE_INDEX = 1;

    // Units / field
    private static final double INCHES_PER_METER = 39.37007874015748;
    private static final double FIELD_CENTER_IN  = 72.0; // (72,72) is center in Pedro

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();

        KalmanFilterParameters p = new KalmanFilterParameters(KAL_MODEL_COV, KAL_DATA_COV);
        kalX = new KalmanFilter(p);
        kalY = new KalmanFilter(p);
        headingErrLP = new LowPassFilter(HEADING_LP_ALPHA);

        loopTimer.reset();
        resetCooldown.reset();

        telemetry.addLine("LL_Test_PedroFusion (FTC axes: +X down, +Y right)");
        telemetry.addLine("A = apply vision -> estimate (gated)");
        telemetry.addLine("B = reset estimate to (72,72,90deg Pedro)");
        telemetry.update();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 0.02;

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;

        /* ---------- RESET ---------- */
        if (b && !lastB) {
            estX = 72.0;
            estY = 72.0;
            estH = Math.toRadians(90.0);
            filtersInit = false;
            resetCooldown.reset();
        }
        lastB = b;

        llResult = limelight.getLatestResult();
        boolean llValid = (llResult != null && llResult.isValid());

        Pose3D mt2 = null;
        Pose3D mt1 = null;

        Pose pedroMT2 = null;
        Pose pedroMT1 = null;

        // Raw Limelight values (FTC): meters + yaw degrees (RZ)
        Double mt1Xm = null, mt1Ym = null, mt1RZ = null;
        Double mt2Xm = null, mt2Ym = null, mt2RZ = null;

        Double mt1FtcYawWrapped = null;
        Double mt1PedroYawDeg   = null;
        Double mt1PedroYawRad   = null;

        if (llValid) {
            mt2 = llResult.getBotpose_MT2();
            mt1 = llResult.getBotpose(); // MT1

            if (mt1 != null) {
                mt1Xm = mt1.getPosition().toUnit(DistanceUnit.METER).x;
                mt1Ym = mt1.getPosition().toUnit(DistanceUnit.METER).y;
                mt1RZ = mt1.getOrientation().getYaw(AngleUnit.DEGREES); // FTC yaw (deg), unwrapped

                mt1FtcYawWrapped = wrapDeg360(mt1RZ);

                // YOUR rule (as last confirmed): Pedro heading (deg) = FTC heading (deg) - 90
                mt1PedroYawDeg = wrapDeg360(mt1FtcYawWrapped - 90.0);
                mt1PedroYawRad = Math.toRadians(mt1PedroYawDeg);

                pedroMT1 = convertFtcMetersToPedro(mt1Xm, mt1Ym, mt1PedroYawRad);
            }

            if (mt2 != null) {
                mt2Xm = mt2.getPosition().toUnit(DistanceUnit.METER).x;
                mt2Ym = mt2.getPosition().toUnit(DistanceUnit.METER).y;
                mt2RZ = mt2.getOrientation().getYaw(AngleUnit.DEGREES);

                double mt2FtcYawWrapped = wrapDeg360(mt2RZ);
                double mt2PedroYawDeg = wrapDeg360(mt2FtcYawWrapped - 90.0);
                double mt2PedroYawRad = Math.toRadians(mt2PedroYawDeg);

                pedroMT2 = convertFtcMetersToPedro(mt2Xm, mt2Ym, mt2PedroYawRad);
            }
        }

        /* ---------- FEED FTC YAW BACK TO LIMELIGHT ---------- */
        // Your rule: feed mt1RZ (FTC yaw) to updateRobotOrientation.
        double ftcYawDegToLL;
        if (mt1RZ != null) {
            ftcYawDegToLL = wrapDeg360(mt1RZ);
        } else {
            // fallback: FTC = Pedro + 90 (inverse of Pedro = FTC - 90)
            ftcYawDegToLL = wrapDeg360(Math.toDegrees(estH) + 90.0);
        }
        limelight.updateRobotOrientation(ftcYawDegToLL);

        boolean haveXY = (pedroMT2 != null);
        boolean haveH  = (pedroMT1 != null);

        if ((haveXY || haveH) && !filtersInit) {
            kalX.reset(estX, 1.0, 0.0);
            kalY.reset(estY, 1.0, 0.0);
            headingErrLP.reset(0.0, 1.0, 0.0);
            filtersInit = true;
        }

        if (filtersInit) {
            if (haveXY) {
                kalX.update(pedroMT2.getX(), estX);
                kalY.update(pedroMT2.getY(), estY);
            }

            if (haveH) {
                double err = wrapRad(pedroMT1.getHeading() - estH);
                headingErrLP.update(err, headingErrLP.getState());
            }
        }

        double filtX = (filtersInit && haveXY) ? kalX.getState() : estX;
        double filtY = (filtersInit && haveXY) ? kalY.getState() : estY;
        double filtH = (filtersInit && haveH)  ? wrapRad(estH + headingErrLP.getState()) : estH;

        /* ---------- APPLY STEP ---------- */
        if (a && !lastA && resetCooldown.seconds() >= RESET_COOLDOWN_S) {
            double dx = filtX - estX;
            double dy = filtY - estY;
            double dpos = Math.hypot(dx, dy);
            double dh = angleDiffRad(filtH, estH);

            if (dpos <= MAX_STEP_IN && dh <= MAX_STEP_RAD) {
                estX = filtX;
                estY = filtY;
                estH = filtH;
                resetCooldown.reset();
            }
        }
        lastA = a;

        /* ---------- TELEMETRY ---------- */
        telemetry.addData("LL valid", llValid);
        telemetry.addData("MT1 exists", mt1 != null);
        telemetry.addData("MT2 exists", mt2 != null);
        telemetry.addData("filtersInit", filtersInit);

        if (mt1 != null) {
            telemetry.addData("MT1 RAW FTC (m) x,y", "%.3f, %.3f", mt1Xm, mt1Ym);
            telemetry.addData("MT1 RAW RZ FTC (deg)", "%.2f", mt1RZ);
            telemetry.addData("MT1 FTC yaw wrapped", "%.2f", mt1FtcYawWrapped);

            telemetry.addData("MT1 Pedro hdg (deg)", "%.2f", mt1PedroYawDeg);
            telemetry.addData("MT1 Pedro hdg (rad)", "%.3f", mt1PedroYawRad);
            telemetry.addData("MT1 Pedro (x,y in)", "%.1f, %.1f", pedroMT1.getX(), pedroMT1.getY());
        } else {
            telemetry.addData("MT1 RAW FTC (m) x,y", "null");
            telemetry.addData("MT1 RAW RZ FTC (deg)", "null");
            telemetry.addData("MT1 FTC yaw wrapped", "null");
            telemetry.addData("MT1 Pedro hdg (deg)", "null");
            telemetry.addData("MT1 Pedro hdg (rad)", "null");
            telemetry.addData("MT1 Pedro (x,y in)", "null");
        }

        telemetry.addData("Pedro MT2 (x,y in)", haveXY
                ? String.format("%.1f, %.1f", pedroMT2.getX(), pedroMT2.getY())
                : "null");

        telemetry.addData("EST Pedro (x,y,deg)", "%.1f, %.1f, %.1f",
                estX, estY, wrapDeg360(Math.toDegrees(estH)));

        telemetry.addData("FTC yaw -> LL", "%.1f", ftcYawDegToLL);
        telemetry.addData("cooldown(s)", "%.2f", resetCooldown.seconds());
        telemetry.update();
    }

    /**
     * FTC -> Pedro (your axis definition):
     * FTC: +X down, +Y right, origin center, meters
     * Pedro: +X right, +Y up, origin bottom-left, inches
     *
     * So:
     *   xPedro = 72 + (yFTC_in)
     *   yPedro = 72 - (xFTC_in)
     */
    private static Pose convertFtcMetersToPedro(double xFtcM, double yFtcM, double headingPedroRad) {
        double xFtcIn = xFtcM * INCHES_PER_METER;
        double yFtcIn = yFtcM * INCHES_PER_METER;

        double xPedro = FIELD_CENTER_IN + yFtcIn;
        double yPedro = FIELD_CENTER_IN - xFtcIn;

        return new Pose(xPedro, yPedro, wrapRad(headingPedroRad));
    }

    private static double wrapRad(double a) {
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        while (a >  Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    private static double angleDiffRad(double a, double b) {
        return Math.abs(wrapRad(a - b));
    }

    private static double wrapDeg360(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }
}
