package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector;
import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector.Provider;

public class ShooterController {

    public interface PoseSupplier { Pose get(); }
    public enum Band { CLOSE, MID, FAR }
    private enum ShootState { IDLE, ARMING, FIRING }

    private static final String MOTOR_A_NAME = "shootMotor1";
    private static final String MOTOR_B_NAME = "shootMotor2";
    private static final String HOOD_SERVO   = "hood";
    private static final String GATE_SERVO   = "gate";

    private static final DcMotorSimple.Direction MOTOR_A_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction MOTOR_B_DIR = DcMotorSimple.Direction.FORWARD;

    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO    = 1.0;

    private static final double CTRL_P = 0.006;
    private static final double CTRL_I = 0.000;
    private static final double CTRL_D = 0.000;
    private static final double CTRL_T = 0.050;
    private static final double CTRL_F = 0.0005;

    private static final double HOOD_CLOSE = 0.72;
    private static final double HOOD_MID   = 0.56;
    private static final double HOOD_FAR   = 0.41;

    private static final double GATE_CLOSED = 0.10;
    private static final double GATE_OPEN   = 0.65;

    private static final double CLOSE_MAX = 26.0;
    private static final double MID_MAX   = 46.0;
    private static final double HYST      = 2.0;

    private static final double RPM_TOL       = 75.0;
    private static final double READY_HOLD_MS = 120.0;

    private static final double MAX_RPM_SLEW_PER_S = 2000.0;

    private final Provider allianceProvider;
    private final PoseSupplier poseSupplier;
    private final DcMotorEx motorA;
    private final DcMotorEx motorB;
    private final Servo hood;
    private final Servo gate;
    private final FilteredPIDFController pidf;

    private Band band = Band.MID;
    private ShootState state = ShootState.IDLE;

    private final ElapsedTime loopDtTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private boolean spinEnabled = false;
    private boolean shootHold = false;

    private double commandedRpm = 0.0;

    private static final double[][] CLOSE_TABLE = {
            {18,1700},{24,1780},{30,1860}
    };
    private static final double[][] MID_TABLE = {
            {32,1920},{38,2020},{44,2120},{50,2240}
    };
    private static final double[][] FAR_TABLE = {
            {52,2320},{58,2450},{64,2590},{70,2740}
    };

    public ShooterController(Provider allianceProvider,
                             PoseSupplier poseSupplier,
                             DcMotorEx motorA,
                             DcMotorEx motorB,
                             Servo hood,
                             Servo gate) {
        this.allianceProvider = allianceProvider;
        this.poseSupplier = poseSupplier;
        this.motorA = motorA;
        this.motorB = motorB;
        this.hood = hood;
        this.gate = gate;

        motorA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorA.setDirection(MOTOR_A_DIR);
        motorB.setDirection(MOTOR_B_DIR);

        FilteredPIDFCoefficients coeffs = new FilteredPIDFCoefficients(CTRL_P, CTRL_I, CTRL_D, CTRL_T, CTRL_F);
        pidf = new FilteredPIDFController(coeffs);

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
                             Servo gate) {
        this(allianceProvider, () -> follower.getPose(), motorA, motorB, hood, gate);
    }

    public static ShooterController create(HardwareMap hw, Follower follower, Provider allianceProvider) {
        DcMotorEx motorA = hw.get(DcMotorEx.class, MOTOR_A_NAME);
        DcMotorEx motorB = hw.get(DcMotorEx.class, MOTOR_B_NAME);
        Servo hood       = hw.get(Servo.class,    HOOD_SERVO);
        Servo gate       = hw.get(Servo.class,    GATE_SERVO);
        return new ShooterController(allianceProvider, follower, motorA, motorB, hood, gate);
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

    private void driveToRpm(double targetRpm) {
        double actual = getShooterRpm();
        double err = targetRpm - actual;
        pidf.updateFeedForwardInput(targetRpm);
        pidf.updateError(err);
        double out = clip(pidf.run(), -1.0, 1.0);
        setPower(out, -out);
    }

    private void setPower(double a, double b) {
        motorA.setPower(a);
        motorB.setPower(b);
    }

    private double getShooterRpm() {
        double rpsA = motorA.getVelocity() / (TICKS_PER_REV * GEAR_RATIO);
        double rpsB = motorB.getVelocity() / (TICKS_PER_REV * GEAR_RATIO);
        return ((rpsA + rpsB) * 0.5) * 60.0;
    }

    private double distanceToGoalInches() {
        Pose p = poseSupplier.get();
        double gx = AllianceSelector.Field.goalX(allianceProvider.getAlliance());
        double gy = AllianceSelector.Field.goalY(allianceProvider.getAlliance());
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
            default:
                return Band.MID;
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
        double[][] T = (b==Band.CLOSE)?CLOSE_TABLE: (b==Band.MID)?MID_TABLE:FAR_TABLE;
        if (d <= T[0][0]) return T[0][1];
        for (int i=0;i<T.length-1;i++){
            double x0=T[i][0], y0=T[i][1], x1=T[i+1][0], y1=T[i+1][1];
            if (d <= x1){
                double t=(d-x0)/(x1-x0);
                return y0 + t*(y1-y0);
            }
        }
        return T[T.length-1][1];
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
                state, allianceProvider.getAlliance(), d, band, spinEnabled?"ON":"OFF", shootHold?"ON":"OFF", commandedRpm, rpm);
    }
}
