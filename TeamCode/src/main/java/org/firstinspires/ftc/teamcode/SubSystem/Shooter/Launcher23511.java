package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDFController;


@Configurable
public class Launcher23511 {

    public static double P = 0.03;
    public static double I = 0.003;
    public static double D = 0.00004;
    public static double F = 0.000039;

    public static double VELOCITY_TOLERANCE = 60;

    public static double BlockerOpenPosition = 0.55;
    public static double BlockerClosedPosition = 0.2;

    public static double MAX_FLYWHEEL_VELOCITY = 1860;
    public static double NOMINAL_VOLTAGE = 12.82;

    public static boolean MOTOR_ONE_REVERSED = false;
    public static boolean MOTOR_TWO_REVERSED = true;

    // Motors & Controller
    private final DcMotorEx flywheelMotorOne;
    private final DcMotorEx flywheelMotorTwo;
    private final VoltageSensor voltageSensor;
    private final PIDFController flywheelController;

    // optional blocker
    private Servo blocker = null;

    // For rising edge detection
    private boolean shootButtonLast = false;
    private boolean headingLock = false;

    public static double targetTPS = 0;

    public static double LutTPS = 800;

    // LUT stuff
    // public static double LUT_TPS_SLOPE = 10.0;
    // public static double LUT_TPS_INTERCEPT = 800.0;
    //
    // private static double computeLutTPS(double distance) {
    //     double tps = LUT_TPS_SLOPE * distance + LUT_TPS_INTERCEPT;
    //     if (tps < 0) tps = 0;
    //     if (tps > MAX_FLYWHEEL_VELOCITY) tps = MAX_FLYWHEEL_VELOCITY;
    //     return tps;
    // }

    public double getTargetTPS() { return targetTPS; }
    public double getCurentRPM() { return flywheelMotorOne.getVelocity(); }

    private enum ShootState {
        IDLE,
        ARMING,
        FIRING
    }

    private ShootState shootState = ShootState.IDLE;

    public Launcher23511(DcMotorEx m1, DcMotorEx m2, VoltageSensor vs) {
        this.flywheelMotorOne = m1;
        this.flywheelMotorTwo = m2;
        this.voltageSensor = vs;

        m1.setDirection(MOTOR_ONE_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        m2.setDirection(MOTOR_TWO_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);

        flywheelController = new PIDFController(P, I, D, F);
        flywheelController.setTolerance(VELOCITY_TOLERANCE);
    }

    public static Launcher23511 create(HardwareMap hw) {
        DcMotorEx m1 = hw.get(DcMotorEx.class, "ShooterA");
        DcMotorEx m2 = hw.get(DcMotorEx.class, "ShooterB");
        VoltageSensor vs = hw.voltageSensor.iterator().next();
        Launcher23511 launcher = new Launcher23511(m1, m2, vs);
        launcher.init();
        return launcher;
    }

    public void init() {
        setFlywheelTicks(0);
        shootState = ShootState.IDLE;
        headingLock = false;
        if (blocker != null) blocker.setPosition(BlockerClosedPosition);
    }

    public double getCurrentRPM() {
        return flywheelMotorOne.getVelocity();
    }

    public void setBlocker(Servo blocker) {
        this.blocker = blocker;
        if (blocker != null) blocker.setPosition(BlockerClosedPosition);
    }

    // still useful for TeleOp code that wants to set it in code
    public void setFlywheelTicks(double tps) {
        if (tps < 0) tps = 0;
        if (tps > MAX_FLYWHEEL_VELOCITY) tps = MAX_FLYWHEEL_VELOCITY;

        targetTPS = tps;  // dashboard & code share this
        // setPoint will be updated every loop in update()
    }

    public void update() {
        // ---- APPLY DASHBOARD / CODE CHANGES TO PID EVERY LOOP ----
        double desiredTPS = targetTPS;
        if (desiredTPS < 0) desiredTPS = 0;
        if (desiredTPS > MAX_FLYWHEEL_VELOCITY) desiredTPS = MAX_FLYWHEEL_VELOCITY;
        targetTPS = desiredTPS;

        flywheelController.setSetPoint(desiredTPS);

        if (desiredTPS <= 0) {
            flywheelMotorOne.setPower(0);
            flywheelMotorTwo.setPower(0);
            return;
        }

        double voltage = voltageSensor.getVoltage();
        double normalizedVoltage = voltage / NOMINAL_VOLTAGE;
        if (normalizedVoltage <= 0) normalizedVoltage = 1;

        flywheelController.setPIDF(P, I, D, F / normalizedVoltage);

        double currentTPS = flywheelMotorOne.getVelocity();
        double power = flywheelController.calculate(currentTPS);

        flywheelMotorOne.setDirection(MOTOR_ONE_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        flywheelMotorTwo.setDirection(MOTOR_TWO_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);

        flywheelMotorOne.setPower(power);
        flywheelMotorTwo.setPower(power);
    }

    public boolean flywheelReady() {
        if (targetTPS <= 0) return false;
        double currentTPS = flywheelMotorOne.getVelocity();
        return Math.abs(targetTPS - currentTPS) <= VELOCITY_TOLERANCE;
    }

    /************************* SHOOTING STATE MACHINE *************************/
    public void updateShooting(boolean shootButton, double x, double y) {
        boolean inZone = isInShootingZone(x, y);

        if (!inZone && shootState != ShootState.IDLE) {
            cancelShooting();
        }

        // Rising edge
        if (shootButton && !shootButtonLast) {
            if (shootState == ShootState.IDLE && inZone) {
                shootState = ShootState.ARMING;

                // LUT
                 //double dx = GOAL_X - x;
                // double dy = GOAL_Y - y;
                // double distance = Math.hypot(dx, dy);
                // LutTPS = computeLutTPS(distance);

                setFlywheelTicks(LutTPS);
                headingLock = true;
            } else {
                cancelShooting();
            }
        }
        shootButtonLast = shootButton;

        switch (shootState) {
            case IDLE:
                headingLock = false;
                if (blocker != null) blocker.setPosition(BlockerClosedPosition);
                setFlywheelTicks(0);
                break;

            case ARMING:
                headingLock = true;
                if (blocker != null) blocker.setPosition(BlockerClosedPosition);
                update();
                if (flywheelReady()) {
                    shootState = ShootState.FIRING;
                }
                break;

            case FIRING:
                headingLock = true;
                update();
                if (blocker != null) blocker.setPosition(BlockerOpenPosition);
                break;
        }
    }

    private void cancelShooting() {
        shootState = ShootState.IDLE;
        headingLock = false;
        setFlywheelTicks(0);
        if (blocker != null) blocker.setPosition(0);
    }

    public boolean isHeadingLockEnabled() {
        return headingLock;
    }

    /************************* SHOOTING ZONE GEOMETRY *************************/
    // Original:
    // public static boolean isInShootingZone(double x, double y) {
    //     return isInBackZone(x, y) || isInFrontZone(x, y);
    // }
    //
    // public static boolean isInBackZone(double x, double y) {
    //     return y <= x - 48 && y <= -x + 96;
    // }
    //
    // public static boolean isInFrontZone(double x, double y) {
    //     return y >= -x + 144 && y >= x;
    // }

    public static double radius = 10.0;

    public static boolean isInShootingZone(double x, double y) {
        return isInBackZone(x, y) || isInFrontZone(x, y);
    }

    public static boolean isInBackZone(double x, double y) {
        double d = radius * 1.41421356237;
        return y <= x - 48 + d && y <= -x + 96 + d;
    }

    public static boolean isInFrontZone(double x, double y) {
        double d = radius * 1.41421356237;
        return y >= -x + 144 - d && y >= x - d;
    }
}
