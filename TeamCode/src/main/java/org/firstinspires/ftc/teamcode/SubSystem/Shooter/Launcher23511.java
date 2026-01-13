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

    private enum ShootState {
        IDLE,
        ARMING,
        FIRING
    }

    private ShootState shootState = ShootState.IDLE;

    private double targetTPS = 0.0;

    // Constructor
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
        if (blocker != null) blocker.setPosition(0);
    }

    public void setBlocker(Servo blocker) {
        this.blocker = blocker;
        if (blocker != null) blocker.setPosition(0);
    }

    public void setFlywheelTicks(double tps) {
        if (tps < 0) tps = 0;
        if (tps > MAX_FLYWHEEL_VELOCITY) tps = MAX_FLYWHEEL_VELOCITY;

        targetTPS = tps;
        flywheelController.setSetPoint(tps);
    }

    public void update() {
        if (targetTPS <= 0) {
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

        // Rising edge
        if (shootButton && !shootButtonLast) {
            if (shootState == ShootState.IDLE && inZone) {
                shootState = ShootState.ARMING;
                headingLock = true;
            } else {
                cancelShooting();
            }
        }
        shootButtonLast = shootButton;

        switch (shootState) {
            case IDLE:
                headingLock = false;
                if (blocker != null) blocker.setPosition(0);
                setFlywheelTicks(0);
                break;

            case ARMING:
                headingLock = true;
                if (blocker != null) blocker.setPosition(0);
                update();

                if (flywheelReady()) {
                    shootState = ShootState.FIRING;
                }
                break;

            case FIRING:
                headingLock = true;
                update();
                if (blocker != null) blocker.setPosition(1);
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
