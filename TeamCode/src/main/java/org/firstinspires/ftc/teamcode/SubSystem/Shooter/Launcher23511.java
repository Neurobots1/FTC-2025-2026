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
    public static double MAX_FLYWHEEL_VELOCITY = 1860;
    public static double DEFAULT_ON_POWER = 0.75;

    public static double NOMINAL_VOLTAGE = 12.82;

    public static boolean MOTOR_ONE_REVERSED = false;
    public static boolean MOTOR_TWO_REVERSED = true;

    // default target when we auto-shoot (you can set from TeleOp too)
    public static double DEFAULT_TARGET_TPS = 400;

    private final DcMotorEx flywheelMotorOne;
    private final DcMotorEx flywheelMotorTwo;
    private final VoltageSensor voltageSensor;
    private final PIDFController flywheelController;

    private double targetVelocityInput = 0.0;

    // Blocker servo (set from TeleOp)
    private Servo blocker = null;

    // Simple state machine for shooting
    private enum ShootState {
        IDLE,      // not shooting
        ARMING,    // spinning up flywheel, waiting to be ready
        FIRING     // flywheel ready, blocker open
    }

    private ShootState shootState = ShootState.IDLE;
    private boolean shootButtonLast = false;   // for rising edge
    private boolean headingLock = false;       // TeleOp can read this

    public Launcher23511(DcMotorEx flywheelMotorOne, DcMotorEx flywheelMotorTwo, VoltageSensor voltageSensor) {
        this.flywheelMotorOne = flywheelMotorOne;
        this.flywheelMotorTwo = flywheelMotorTwo;
        this.voltageSensor = voltageSensor;

        flywheelMotorOne.setDirection(MOTOR_ONE_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        flywheelMotorTwo.setDirection(MOTOR_TWO_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);

        flywheelController = new PIDFController(P, I, D, F);
        flywheelController.setTolerance(VELOCITY_TOLERANCE);
    }

    public static Launcher23511 create(HardwareMap hardwareMap) {
        DcMotorEx flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        DcMotorEx flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        Launcher23511 launcher = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        launcher.init();
        return launcher;
    }

    public void init() {
        setFlywheelTicks(0);
        if (blocker != null) blocker.setPosition(1); // closed
        shootState = ShootState.IDLE;
        headingLock = false;
    }

    // Let TeleOp give us the blocker servo
    public void setBlocker(Servo blocker) {
        this.blocker = blocker;
        if (this.blocker != null) {
            this.blocker.setPosition(1); // closed by default
        }
    }

    // velInput is interpreted directly as ticksPerSecond (legacy API)


    public void setFlywheelTicks(double ticksPerSecond) {
        if (ticksPerSecond < 0) ticksPerSecond = 0;
        if (ticksPerSecond > MAX_FLYWHEEL_VELOCITY) ticksPerSecond = MAX_FLYWHEEL_VELOCITY;

        targetVelocityInput = ticksPerSecond;
        flywheelController.setSetPoint(ticksPerSecond);
    }

    // Low-level flywheel control (PID + voltage comp)
    public void update() {
        double setPoint = flywheelController.getSetPoint();

        // If target is 0, just make sure we're off
        if (setPoint == 0) {
            flywheelMotorOne.setPower(0);
            flywheelMotorTwo.setPower(0);
            return;
        }

        double voltage = voltageSensor.getVoltage();
        double normalizedVoltage = voltage / NOMINAL_VOLTAGE;
        if (normalizedVoltage <= 0) normalizedVoltage = 1.0;

        flywheelController.setPIDF(P, I, D, F / normalizedVoltage);

        flywheelMotorOne.setDirection(MOTOR_ONE_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        flywheelMotorTwo.setDirection(MOTOR_TWO_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);

            double currentVelocity = flywheelMotorOne.getVelocity();
            double power = flywheelController.calculate(currentVelocity);
            flywheelMotorOne.setPower(power);
            flywheelMotorTwo.setPower(power);

            // fallback "on" behavior without PIDF (if you ever use it
        }


    public void stop() {
        setFlywheelTicks(0);
        flywheelMotorOne.setPower(0);
        flywheelMotorTwo.setPower(0);
    }

    public boolean flywheelReady() {

        double targetTPS = flywheelController.getSetPoint();
        if (targetTPS <= 0) return false;

        double currentTPS = flywheelMotorOne.getVelocity();

        return Math.abs(targetTPS - currentTPS) <= VELOCITY_TOLERANCE;
    }

    public double getCurrentVelocity() {
        return flywheelMotorOne.getVelocity();
    }

    public double getTargetTicksPerSecond() {
        return flywheelController.getSetPoint();
    }




    /************************* Shooting state machine ***************************************/
    public void updateShooting(boolean shootButton, double x, double y) {
        boolean inZone = isInShootingZone(x, y);


        // Rising edge on shoot button
        if (shootButton && !shootButtonLast) {
            switch (shootState) {
                case IDLE:
                    // only enter shooting if we are in zone
                    if (inZone) {
                        shootState = ShootState.ARMING;
                        headingLock = true;
                        setFlywheelTicks(DEFAULT_ON_POWER);
                    }
                    break;

                case ARMING:
                case FIRING:
                    // any press while active cancels
                    cancelShooting();
                    break;
            }
        }
        shootButtonLast = shootButton;

        // State actions
        switch (shootState) {
            case IDLE:
                headingLock = false;
                if (blocker != null) blocker.setPosition(0); // closed
                stop();
                break;

            case ARMING:
                headingLock = true;
                if (blocker != null) blocker.setPosition(0); // closed while spinning up
                update(); // run PID

                if (flywheelReady()) {
                    shootState = ShootState.FIRING;
                }
                break;

            case FIRING:
                headingLock = true;
                update(); // keep speed
                if (blocker != null) blocker.setPosition(1); // fire
                break;
        }
    }

    private void cancelShooting() {
        shootState = ShootState.IDLE;
        headingLock = false;
        stop();
        if (blocker != null) blocker.setPosition(1);
    }

    public boolean isHeadingLockEnabled() {
        return headingLock;
    }

    // ------------- Shooting zone logic -------------

    public static boolean isInShootingZone(double x, double y) {
        return isInBackZone(x, y) || isInFrontZone(x, y);
    }

    public static boolean isInBackZone(double x, double y) {
        return y <= x - 48 && y <= -x + 96;
    }

    public static boolean isInFrontZone(double x, double y) {
        return y >= -x + 144 && y >= x;
    }
}
