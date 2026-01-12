package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector;
import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector.Provider;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LUTs;

@Configurable
public class ShooterController {

    public interface PoseSupplier { Pose get(); }
    public enum Band { CLOSE, MID, FAR }
    private enum ShootState { IDLE, ARMING, FIRING }

    private static final String MOTOR_A_NAME = "ShooterA";
    private static final String MOTOR_B_NAME = "ShooterB";
    private static final String HOOD_SERVO   = "hoodServo";
    private static final String GATE_SERVO   = "gateServo";

    // Use DcMotorSimple.Direction enum
    private static final DcMotorSimple.Direction MOTOR_A_DIR = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction MOTOR_B_DIR = DcMotorSimple.Direction.REVERSE;

    // Encoder -> output shaft
    private static final double TICKS_PER_REV = 28.0; // ticks per output rev
    private static final double GEAR_RATIO    = 1.0;  // encoder -> wheel ratio (you said 1:1)

    // Panels-editable gains (kept public static for your Panels)
    public static double CTRL_P = 0.0001;
    public static double CTRL_I = 0;
    public static double CTRL_D = 0.05;
    public static double CTRL_F = 0; // will be voltage-compensated internally

    public static double HOOD_CLOSE = 1;
    public static double HOOD_MID   = 0.5;
    public static double HOOD_FAR   = 0;

    private static final double GATE_CLOSED = 0.10;
    private static final double GATE_OPEN   = 0.65;

    private static final double CLOSE_MAX = 26.0;
    private static final double MID_MAX   = 46.0;
    private static final double HYST      = 2.0;

    public static double TARGET_RPM = 0;

    private static final double RPM_TOL       = 75.0;
    private static final double READY_HOLD_MS = 120.0;

    private static final double MAX_RPM_SLEW_PER_S = 2000.0;

    private static final double CLOSE_MIN_D = 0,  CLOSE_MAX_D = 30.0;
    private static final double MID_MIN_D   = 30, MID_MAX_D   = 50.0;
    private static final double FAR_MIN_D   = 50, FAR_MAX_D   = 70.0;

    private final Provider allianceProvider;
    private final PoseSupplier poseSupplier;
    private final DcMotorEx motorA;
    private final DcMotorEx motorB;
    private final Servo hood;
    private final Servo gate;
    private final PIDFController pidf;
    private final VoltageSensor voltageSensor; // for F voltage compensation

    private Band band = Band.MID;
    private ShootState state = ShootState.IDLE;

    private final ElapsedTime loopDtTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private boolean spinEnabled = false;
    private boolean shootHold = false;

    private double commandedRpm = 0.0;

    public ShooterController(Provider allianceProvider,
                             PoseSupplier poseSupplier,
                             DcMotorEx motorA,
                             DcMotorEx motorB,
                             Servo hood,
                             Servo gate,
                             VoltageSensor voltageSensor) {
        this.allianceProvider = allianceProvider;
        this.poseSupplier = poseSupplier;
        this.motorA = motorA;
        this.motorB = motorB;
        this.hood = hood;
        this.gate = gate;
        this.voltageSensor = voltageSensor;

        motorA.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorA.setDirection(MOTOR_A_DIR);
        motorB.setDirection(MOTOR_B_DIR);
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidf = new PIDFController(new PIDFCoefficients(CTRL_P, CTRL_I, CTRL_D, CTRL_F));

        hood.setPosition(HOOD_MID);
        gate.setPosition(GATE_CLOSED);

        loopDtTimer.reset();
        readyTimer.reset();
    }

    public ShooterController(Provider allianceProvider,
                             Follower follower,
                             DcMotorEx motorA,
                             DcMotorEx motorB,
                             Servo hood,
                             Servo gate,
                             VoltageSensor voltageSensor) {
        this(allianceProvider,
                follower != null ? () -> follower.getPose() : () -> new Pose(0,0,0),
                motorA, motorB, hood, gate, voltageSensor);
    }

    public static ShooterController create(HardwareMap hw, Follower follower, AllianceSelector.Alliance alliance) {
        VoltageSensor vs = null;
        for (VoltageSensor v : hw.getAll(VoltageSensor.class)) { vs = v; break; }

        return new ShooterController(
                () -> alliance,
                follower,
                hw.get(DcMotorEx.class, MOTOR_A_NAME),
                hw.get(DcMotorEx.class, MOTOR_B_NAME),
                hw.get(Servo.class, HOOD_SERVO),
                hw.get(Servo.class, GATE_SERVO),
                vs
        );
    }

    public void setSpinEnabled(boolean on) {
        spinEnabled = on;
        if (!on) {
            commandedRpm = 0;
            driveToRpm(0);
            gate.setPosition(GATE_CLOSED);
            state = ShootState.IDLE;
            readyTimer.reset();
        }
    }

    public void setShootHold(boolean on) { shootHold = on; }
    public AllianceSelector.Alliance getAlliance() { return allianceProvider.getAlliance(); }

    public void update() {
        double dt = Math.max(1e-3, loopDtTimer.seconds());
        loopDtTimer.reset();

        double d = distanceToGoalInches();
        Band newBand = decideBand(d);
        if (newBand != band) band = newBand;
        hood.setPosition(hoodPos(band));

        double targetRpm = spinEnabled ? rpmFor(band, d) : 0.0;

        if (!spinEnabled) {
            commandedRpm = slewRpm(commandedRpm, 0.0, dt);
            driveToRpm(commandedRpm);
            gate.setPosition(GATE_CLOSED);
            state = ShootState.IDLE;
            return;
        }

        commandedRpm = slewRpm(commandedRpm, targetRpm, dt);
        driveToRpm(commandedRpm);

        boolean ready = atSpeed(targetRpm);
        if (ready) {
            if (readyTimer.milliseconds() == 0) readyTimer.reset();
        } else {
            readyTimer.reset();
        }

        boolean allowFire = shootHold && ready && readyTimer.milliseconds() >= READY_HOLD_MS;

        if (allowFire) {
            gate.setPosition(GATE_OPEN);
            state = ShootState.FIRING;
        } else {
            gate.setPosition(GATE_CLOSED);
            state = ShootState.ARMING;
        }
    }

    // ---------- Core speed control (ticks/sec inside) ----------

    public void driveToRpm(double targetRpm) {
        // Clean stop at zero target
        if (targetRpm <= 0) {
            setPower(0, 0);
            // re-apply current coeffs to clear internal accumulators in Pedro PIDF
            pidf.setCoefficients(new PIDFCoefficients(CTRL_P, CTRL_I, CTRL_D, CTRL_F));
            return;
        }

        // Voltage-compensate F so baseline power stays correct as battery sags
        double v = getBatteryVoltageSafe();
        double Fscaled = CTRL_F * (12.0 / Math.max(9.0, v));
        pidf.setCoefficients(new PIDFCoefficients(CTRL_P, CTRL_I, CTRL_D, Fscaled));

        // Work in ticks/sec for both setpoint and measurement
        double targetTicks = rpmToTicksPerSec(targetRpm);
        double actualTicks = getVelocityTicksPerSec();
        double errorTicks  = targetTicks - actualTicks;

        pidf.updateFeedForwardInput(targetTicks);
        pidf.updateError(errorTicks);
        double out = clip(pidf.run(), -1.0, 1.0);

        // Counter-rotation: send opposite power signs
        setPower(out, -out);
    }

    private double getVelocityTicksPerSec() {
        return motorA.getVelocity(); // signed ticks/sec from SDK
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_REV / 60.0) / GEAR_RATIO;
    }

    private double getBatteryVoltageSafe() {
        if (voltageSensor != null) {
            double v = voltageSensor.getVoltage();
            return (v > 0) ? v : 12.0;
        }
        return 12.0;
    }

    private void setPower(double a, double b) {
        motorA.setPower(a);
        motorB.setPower(b);
    }

    public void setPIDF(double p, double i, double d, double f) {
        pidf.setCoefficients(new PIDFCoefficients(p, i, d, f));
    }

    public void tuneRPM(double rpm) {
        driveToRpm(rpm);
    }

    // Signed RPM for telemetry
    public double getShooterRpm() {
        return getVelocityTicksPerSec() * 60.0 / TICKS_PER_REV;
    }

    // ---------- Helpers for band/hood ----------

    private double distanceToGoalInches() {
        Pose p = poseSupplier.get();
        double gx = org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector.Field.goalX(allianceProvider.getAlliance());
        double gy = org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector.Field.goalY(allianceProvider.getAlliance());
        double dx = gx - p.getX();
        double dy = gy - p.getY();
        return Math.hypot(dx, dy);
    }

    private Band decideBand(double d) {
        switch (band) {
            case CLOSE:
                if (d > CLOSE_MAX + HYST) return (d <= MID_MAX) ? Band.MID : Band.FAR;
                return Band.CLOSE;
            case MID:
                if (d <= CLOSE_MAX - HYST) return Band.CLOSE;
                if (d > MID_MAX + HYST)    return Band.FAR;
                return Band.MID;
            case FAR:
                if (d <= MID_MAX - HYST)   return (d <= CLOSE_MAX) ? Band.CLOSE : Band.MID;
                return Band.FAR;
            default: return Band.MID;
        }
    }

    private double hoodPos(Band b) {
        switch (b) {
            case CLOSE: return HOOD_CLOSE;
            case MID:   return HOOD_MID;
            case FAR:   return HOOD_FAR;
            default:    return HOOD_MID;
        }
    }

    private double rpmFor(Band b, double d) {
        double x = clampToBand(b, d);
        switch (b) {
            case CLOSE: return LUTs.closeRPM.get(x);
            case MID:   return LUTs.midRPM.get(x);
            case FAR:   return LUTs.farRPM.get(x);
        }
        return 0.0;
    }

    private double clampToBand(Band b, double d) {
        switch (b) {
            case CLOSE: return clip(d, CLOSE_MIN_D, CLOSE_MAX_D);
            case MID:   return clip(d, MID_MIN_D, MID_MAX_D);
            case FAR:   return clip(d, FAR_MIN_D, FAR_MAX_D);
            default:    return d;
        }
    }

    private boolean atSpeed(double targetRpm) {
        return Math.abs(getShooterRpm() - targetRpm) <= RPM_TOL;
    }

    private double slewRpm(double currentCmd, double target, double dtSeconds) {
        double maxStep = MAX_RPM_SLEW_PER_S * dtSeconds;
        double err = target - currentCmd;
        if (Math.abs(err) <= maxStep) return target;
        return currentCmd + Math.copySign(maxStep, err);
    }

    private static double clip(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }

    public String telemetryLine() {
        double d = distanceToGoalInches();
        double rpm = getShooterRpm();
        return String.format("Shooter[%s|%s] d=%.1f in, band=%s, spin=%s, hold=%s, cmdRPM=%.0f, actRPM=%.0f",
                state, allianceProvider.getAlliance(), d, band,
                spinEnabled ? "ON":"OFF", shootHold ? "ON":"OFF",
                commandedRpm, rpm);
    }

    public void addTuningTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        double actualRpm = getShooterRpm();
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Actual RPM", "%.1f", actualRpm);
        telemetry.addData("Error RPM", "%.1f", (TARGET_RPM - actualRpm));
        telemetry.addData("P", CTRL_P);
        telemetry.addData("I", CTRL_I);
        telemetry.addData("D", CTRL_D);
        telemetry.addData("F (raw)", CTRL_F);
        telemetry.addData("F (12V adj)", CTRL_F * (12.0 / Math.max(9.0, getBatteryVoltageSafe())));
        telemetry.addData("A vel (ticks/s)", "%.1f", getVelocityTicksPerSec());
        telemetry.addData("Batt V", "%.2f", getBatteryVoltageSafe());
    }
}
