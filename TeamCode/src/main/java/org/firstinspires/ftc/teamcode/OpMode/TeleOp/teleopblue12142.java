package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import static org.firstinspires.ftc.teamcode.OpMode.Autonomous.Auto_Blue1.finalPose;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Launcher23511;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.Relocalisation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "teleopblueNOHEADING", group = "Tuning")
public class teleopblue12142 extends OpMode {

    public static double rawPower = -1;

    public static boolean usePIDF = true;
    public static boolean rawPowerMode = false;
    public static boolean shooterEnabled = false;
    public static double targetTicksPerSecond = 0;
    public static double testPower = 1.0;
    private JoinedTelemetry jt;

    private boolean ready = false;
    private Follower follower;
    private DcMotorEx intake;
    private final Pose startingPose = new Pose(finalPose.getX(), finalPose.getY(), finalPose.getHeading());
    private static final double GOAL_X = 12;
    private static final double GOAL_Y = 132;
    private final Pose goalPose = new Pose(12, 132, 0.0);
    // 🔹 This stays the same, but now uses Hermite inside
    private ShooterTicksLUT shooterLUT = new ShooterTicksLUT();

    private final InterpLUT lut = new InterpLUT();

    private Launcher23511 launcher;
    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;
    private VoltageSensor voltageSensor;
    private TelemetryManager telemetryManager;
    private IntakeMotor intkM;
    private Robot init;
    private AprilTagPipeline aprilTagPipeline;
    private Relocalisation relocaliser;

    private boolean lastOptions = false;



    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        jt = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        intkM = new IntakeMotor(hardwareMap);
        init = new Robot(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcher = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        aprilTagPipeline = new AprilTagPipeline(hardwareMap); // <-- replace with YOUR pipeline creation if different
        relocaliser = new Relocalisation(hardwareMap, aprilTagPipeline);

        launcher.init();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        if (gamepad1.options) {
            Pose tagPose = relocaliser.relocalisation();
            if (tagPose != null) {
                follower.setPose(tagPose);
            }
        }


        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                0.75 * -gamepad1.right_stick_x,
                true
        );

        if (gamepad1.right_bumper) intkM.intake();
        else if (gamepad1.left_bumper) intkM.slowOuttake();
        else intkM.stop();

        if (gamepad1.dpad_up) {
            rawPowerMode = false;
            usePIDF = true;
        }
        if (gamepad1.dpad_down) {
            usePIDF = false;
            rawPowerMode = true;
        }
        if (gamepad1.a) shooterEnabled = true;
        if (gamepad1.b) shooterEnabled = false;


        if (rawPowerMode) {
            flywheelMotorOne.setPower(rawPower);
            flywheelMotorTwo.setPower(rawPower);
        } else {
            if (shooterEnabled) {
                targetTicksPerSecond = shooterLUT.getTicksForDistance(distanceToGoal());
                launcher.setFlywheelTicks(targetTicksPerSecond);
            } else {
                launcher.setFlywheelTicks(0);
            }
            launcher.update();
        }


        if (gamepad1.share){
            Pose middlePose = new Pose(follower.getPose().getX(),follower.getPose().getY(),90);
            follower.setPose(middlePose);
        }

        double currentVelocity = flywheelMotorOne.getVelocity();

        telemetryManager.debug("shooterEnabled", shooterEnabled);
        telemetryManager.debug("targetTicksPerSecond", targetTicksPerSecond);
        telemetryManager.debug("testPower", testPower);
        telemetryManager.debug("currentVelocity", currentVelocity);
        telemetryManager.debug("P", Launcher23511.P);
        telemetryManager.debug("I", Launcher23511.I);
        telemetryManager.debug("D", Launcher23511.D);
        telemetryManager.debug("F", Launcher23511.F);
        telemetryManager.debug("NOMINAL_VOLTAGE", Launcher23511.NOMINAL_VOLTAGE);
        telemetryManager.debug("pose2D", follower.getPose());
        telemetryManager.debug("distance goal", distanceToGoal());
        jt.addData("targetTicksPerSecond", "%.0f", targetTicksPerSecond);
        jt.addData("currentVelocity", "%.0f", currentVelocity);
        jt.addData("positon", follower.getPose());
        jt.addData("Power", intake.getPower());
        jt.addData("Shooter", shooterEnabled);
        jt.addData("Reverse", rawPowerMode);
        jt.addData("PID", usePIDF);
        jt.addData("ready", launcher.flywheelReady());
        jt.update();
        telemetryManager.update();
    }

    // hermit lut with conversion
    private static class ShooterTicksLUT {

        // Distances in SAME UNITS as follower.getPose()
        private final double[] distances = {
                44, 51, 63, 71, 91, 101, 137
        };

        // Matching flywheel target velocities in ticks/second
        private final double[] ticks = {
                600, 650, 700, 750, 800, 850, 970
        };

        private final CubicHermiteInterpolator hermite;

        ShooterTicksLUT() {
            hermite = new CubicHermiteInterpolator(distances, ticks);
        }

        public double getTicksForDistance(double d) {
            return hermite.interpolate(d);
        }

        // Monotonic Cubic Hermite Interpolator
        private static class CubicHermiteInterpolator {
            private final double[] x;
            private final double[] y;
            private final double[] m; // slopes

            CubicHermiteInterpolator(double[] x, double[] y) {
                if (x.length != y.length) {
                    throw new IllegalArgumentException("Arrays must be same length");
                }
                this.x = x;
                this.y = y;
                this.m = computeSlopes(x, y);
            }

            private double[] computeSlopes(double[] x, double[] y) {
                int n = x.length;
                double[] m = new double[n];
                double[] d = new double[n - 1];

                // secant slopes between points
                for (int i = 0; i < n - 1; i++) {
                    d[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
                }

                // initial tangents
                m[0] = d[0];
                for (int i = 1; i < n - 1; i++) {
                    if (d[i - 1] * d[i] <= 0) {
                        m[i] = 0.0; // slope sign change → flatten
                    } else {
                        m[i] = 0.5 * (d[i - 1] + d[i]);
                    }
                }
                m[n - 1] = d[n - 2];

                // enforce monotonicity more precise
                for (int i = 0; i < n - 1; i++) {
                    if (Math.abs(d[i]) < 1e-9) {
                        m[i] = 0.0;
                        m[i + 1] = 0.0;
                    } else {
                        double a = m[i] / d[i];
                        double b = m[i + 1] / d[i];
                        double r = a * a + b * b;
                        if (r > 9.0) {
                            double t = 3.0 / Math.sqrt(r);
                            m[i] = t * a * d[i];
                            m[i + 1] = t * b * d[i];
                        }
                    }
                }
                return m;
            }

            public double interpolate(double xVal) {
                int n = x.length;

                // clamp out of range
                if (xVal <= x[0]) return y[0];
                if (xVal >= x[n - 1]) return y[n - 1];

                // find interval
                int i = 0;
                while (i < n - 1 && xVal > x[i + 1]) i++;

                double h = x[i + 1] - x[i];
                double t = (xVal - x[i]) / h;

                double y0 = y[i];
                double y1 = y[i + 1];
                double m0 = m[i] * h;
                double m1 = m[i + 1] * h;

                double t2 = t * t;
                double t3 = t2 * t;

                double h00 = 2 * t3 - 3 * t2 + 1;
                double h10 = t3 - 2 * t2 + t;
                double h01 = -2 * t3 + 3 * t2;
                double h11 = t3 - t2;

                return h00 * y0 + h10 * m0 + h01 * y1 + h11 * m1;
            }
        }
    }

    public double distanceToGoal() {
        double goalX = 12;
        double goalY = 136;
        double dx = goalX - follower.getPose().getX();
        double dy = goalY - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }
}
