package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public class ShooterTuner {

    // ----------------- Alliance / Goal -----------------
    public enum Alliance { BLUE, RED }
    private Alliance alliance = Alliance.BLUE;

    // Put your real goal coords here (in Pedro field units)
    // If you already have exact goal coords in your other code, paste them and I’ll set them exactly.
    public static double GOAL_X_BLUE = 0;
    public static double GOAL_Y_BLUE = 138;

    public static double GOAL_X_RED  = 138;
    public static double GOAL_Y_RED  = 138;

    // ----------------- Follower / Pose -----------------
    private Follower follower;
    public static double START_X = 72;
    public static double START_Y = 72;
    public static double START_HEADING_DEG = 90;

    // ----------------- Shooter Tunables -----------------
    public static double P = 0.03;
    public static double I = 0.003;
    public static double D = 0.00004;
    public static double F = 0.000039;

    public static double VELOCITY_TOLERANCE = 60;

    public static double MAX_FLYWHEEL_VELOCITY = 1860;
    public static double NOMINAL_VOLTAGE = 12.82;

    public static boolean MOTOR_ONE_REVERSED = false;
    public static boolean MOTOR_TWO_REVERSED = true;

    public static double BLOCKER_OPEN_POS = 0.55;
    public static double BLOCKER_CLOSED_POS = 0.25;

    public static double BLOCKER_OPEN_DELAY_S = 0.30;

    // Driver-chosen TPS
    public static double MANUAL_TPS = 850;
    public static double TPS_STEP = 10;

    // ----------------- Hardware -----------------
    private DcMotorEx motorA;
    private DcMotorEx motorB;
    private Servo blocker;
    private VoltageSensor voltageSensor;

    // ----------------- Control -----------------
    private PIDFController pid;

    private enum ShootState { IDLE, ARMING, OPENING_BLOCKER, FIRING }
    private ShootState shootState = ShootState.IDLE;

    private double targetTPS = 0;
    private boolean shootButtonLast = false;

    private final ElapsedTime blockerTimer = new ElapsedTime();
    private boolean blockerOpenCommanded = false;

    private final ElapsedTime adjustTimer = new ElapsedTime();

    // ----------------- Init -----------------
    public void init(HardwareMap hw) {
        // follower
        follower = Constants.createFollower(hw);
        follower.update();

        // shooter hardware
        motorA = hw.get(DcMotorEx.class, "ShooterA");
        motorB = hw.get(DcMotorEx.class, "ShooterB");
        blocker = hw.get(Servo.class, "Blocker");
        voltageSensor = hw.voltageSensor.iterator().next();

        motorA.setDirection(MOTOR_ONE_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        motorB.setDirection(MOTOR_TWO_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);

        pid = new PIDFController(P, I, D, F);
        pid.setTolerance(VELOCITY_TOLERANCE);

        commandBlockerClosed();
        setTargetTPS(0);

        adjustTimer.reset();
    }

    public void setAlliance(Alliance a) {
        alliance = a;
    }

    public void startTeleop() {
        if (follower == null) return;

        Pose startPose = new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEG)
        );

        follower.startTeleopDrive();
        follower.setStartingPose(startPose);
    }

    // ----------------- Main loop -----------------
    public void updateLoop(double driveY, double driveX, double turn,
                           boolean fieldCentric, double fieldCentricOffset,
                           boolean shootToggleButton,
                           boolean tpsUpButton, boolean tpsDownButton) {

        // update follower
        follower.update();
        follower.setTeleOpDrive(driveY, driveX, turn, fieldCentric, Math.toRadians(180));

        // adjust TPS (debounced)
        if (adjustTimer.milliseconds() > 120) {
            if (tpsUpButton) {
                MANUAL_TPS += TPS_STEP;
                adjustTimer.reset();
            } else if (tpsDownButton) {
                MANUAL_TPS -= TPS_STEP;
                adjustTimer.reset();
            }
        }
        MANUAL_TPS = clamp(MANUAL_TPS, 0, MAX_FLYWHEEL_VELOCITY);

        // edge toggle
        if (shootToggleButton && !shootButtonLast) {
            if (shootState == ShootState.IDLE) {
                shootState = ShootState.ARMING;
                setTargetTPS(MANUAL_TPS);
            } else {
                cancelShooting();
            }
        }
        shootButtonLast = shootToggleButton;

        // shooter state machine
        switch (shootState) {
            case IDLE:
                commandBlockerClosed();
                setTargetTPS(0);
                break;

            case ARMING:
                commandBlockerClosed();
                setTargetTPS(MANUAL_TPS);
                if (atSpeed()) {
                    commandBlockerOpen();
                    shootState = ShootState.OPENING_BLOCKER;
                }
                break;

            case OPENING_BLOCKER:
                setTargetTPS(MANUAL_TPS);
                commandBlockerOpen();
                if (flywheelReady()) {
                    shootState = ShootState.FIRING;
                }
                break;

            case FIRING:
                setTargetTPS(MANUAL_TPS);
                commandBlockerOpen();
                break;
        }


        updateFlywheel();

    }

    // ----------------- Distance / Pose -----------------
    public Pose getPose() {
        return follower.getPose();
    }

    public double getDistanceToGoal() {
        Pose p = follower.getPose();
        double gx = (alliance == Alliance.BLUE) ? GOAL_X_BLUE : GOAL_X_RED;
        double gy = (alliance == Alliance.BLUE) ? GOAL_Y_BLUE : GOAL_Y_RED;
        return Math.hypot(gx - p.getX(), gy - p.getY());
    }

    // ----------------- Shooter telemetry helpers -----------------
    public double getTargetTPS() { return targetTPS; }
    public double getCurrentTPS() { return motorA.getVelocity(); }
    public String getShootState() { return shootState.name(); }
    public boolean isAtSpeed() { return atSpeed(); }

    // ----------------- Internal helpers -----------------
    private void setTargetTPS(double tps) {
        targetTPS = clamp(tps, 0, MAX_FLYWHEEL_VELOCITY);
        pid.setSetPoint(targetTPS);
    }

    private void updateFlywheel() {
        double voltage = voltageSensor.getVoltage();
        double norm = (voltage <= 0) ? 1.0 : voltage / NOMINAL_VOLTAGE;
        if (norm <= 0) norm = 1.0;

        pid.setPIDF(P, I, D, F / norm);

        if (targetTPS <= 0) {
            motorA.setPower(0);
            motorB.setPower(0);
            return;
        }

        double currentTPS = motorA.getVelocity();
        double power = pid.calculate(currentTPS);

        motorA.setDirection(MOTOR_ONE_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        motorB.setDirection(MOTOR_TWO_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);

        motorA.setPower(power);
        motorB.setPower(power);
    }

    private boolean atSpeed() {
        if (targetTPS <= 0) return false;
        return Math.abs(targetTPS - motorA.getVelocity()) <= VELOCITY_TOLERANCE;
    }

    private boolean flywheelReady() {
        if (!atSpeed()) return false;
        if (!blockerOpenCommanded) return false;
        return blockerTimer.seconds() >= BLOCKER_OPEN_DELAY_S;
    }

    private void commandBlockerOpen() {
        blocker.setPosition(BLOCKER_OPEN_POS);
        if (!blockerOpenCommanded) {
            blockerOpenCommanded = true;
            blockerTimer.reset();
        }
    }

    private void commandBlockerClosed() {
        blocker.setPosition(BLOCKER_CLOSED_POS);
        blockerOpenCommanded = false;
    }

    private void cancelShooting() {
        shootState = ShootState.IDLE;
        setTargetTPS(0);
        commandBlockerClosed();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
