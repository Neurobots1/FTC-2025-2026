package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Configurable
public class LauncherSubsystem {

    public static double P = 0.03;
    public static double I = 0.003;
    public static double D = 0.00004;
    public static double F = 0.000039;

    public static double VELOCITY_TOLERANCE = 60;

    public static double BlockerOpenPosition = 0.55;
    public static double BlockerClosedPosition = 0.25;

    public static double MAX_FLYWHEEL_VELOCITY = 1860;
    public static double NOMINAL_VOLTAGE = 12.82;

    public static boolean MOTOR_ONE_REVERSED = false;
    public static boolean MOTOR_TWO_REVERSED = true;

    public static double BLOCKER_OPEN_DELAY_S = 0.3;

    private final DcMotorEx flywheelMotorOne;
    private final DcMotorEx flywheelMotorTwo;
    private final VoltageSensor voltageSensor;
    private final PIDFController flywheelController;

    private Servo blocker = null;

    private boolean shootButtonLast = false;
    private boolean headingLock = false;

    public static double targetTPS = 0;
    public static double LutTPS = 800;

    private final ElapsedTime blockerTimer = new ElapsedTime();
    private boolean blockerOpenCommanded = false;

    // Far zone TPS is a constant (tune this)
    public static double FAR_ZONE_TPS = 820;

    // Close zone: y = 399.5011 + 4.804155*x - 0.00739844*x^2
    // y is RPM/TPS, x is distance
    private static double computeLutTPSClose(double distance) {
        double tps = 446.5017
                + 1.199221 * distance
                + 0.007467123 * distance * distance;

        if (tps < 0) tps = 0;
        if (tps > MAX_FLYWHEEL_VELOCITY) tps = MAX_FLYWHEEL_VELOCITY;
        return tps;
    }


    // Far zone: constant TPS
    private static double computeLutTPSFar() {
        double tps = FAR_ZONE_TPS;

        if (tps < 0) tps = 0;
        if (tps > MAX_FLYWHEEL_VELOCITY) tps = MAX_FLYWHEEL_VELOCITY;
        return tps;
    }

    private static double computeLutTPS(double distance, double x, double y) {
        if (isInBackZone(x, y)) return computeLutTPSFar();       // far zone
        return computeLutTPSClose(distance);                      // close zone (front/default)
    }

    public double getTargetTPS() { return targetTPS; }
    public double getCurentRPM() { return flywheelMotorOne.getVelocity(); }

    private enum ShootState {
        IDLE,
        ARMING,
        OPENING_BLOCKER,
        FIRING
    }

    private ShootState shootState = ShootState.IDLE;

    public LauncherSubsystem(DcMotorEx m1, DcMotorEx m2, VoltageSensor vs) {
        this.flywheelMotorOne = m1;
        this.flywheelMotorTwo = m2;
        this.voltageSensor = vs;

        m1.setDirection(MOTOR_ONE_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        m2.setDirection(MOTOR_TWO_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);

        flywheelController = new PIDFController(P, I, D, F);
        flywheelController.setTolerance(VELOCITY_TOLERANCE);
    }

    public static LauncherSubsystem create(HardwareMap hw) {
        DcMotorEx m1 = hw.get(DcMotorEx.class, "ShooterA");
        DcMotorEx m2 = hw.get(DcMotorEx.class, "ShooterB");
        VoltageSensor vs = hw.voltageSensor.iterator().next();
        LauncherSubsystem launcher = new LauncherSubsystem(m1, m2, vs);
        launcher.init();
        return launcher;
    }

    public void init() {
        setFlywheelTicks(0);
        shootState = ShootState.IDLE;
        headingLock = false;
        commandBlockerClosed();
    }

    public double getCurrentRPM() {
        return flywheelMotorOne.getVelocity();
    }

    public void setBlocker(Servo blocker) {
        this.blocker = blocker;
        commandBlockerClosed();
    }

    public void setFlywheelTicks(double tps) {
        if (tps < 0) tps = 0;
        if (tps > MAX_FLYWHEEL_VELOCITY) tps = MAX_FLYWHEEL_VELOCITY;
        targetTPS = tps;
    }

    private void commandBlockerOpen() {
        if (blocker != null) blocker.setPosition(BlockerOpenPosition);
        if (!blockerOpenCommanded) {
            blockerOpenCommanded = true;
            blockerTimer.reset();
        }
    }

    private void commandBlockerClosed() {
        if (blocker != null) blocker.setPosition(BlockerClosedPosition);
        blockerOpenCommanded = false;
    }

    public void update() {
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
        boolean atSpeed = Math.abs(targetTPS - currentTPS) <= VELOCITY_TOLERANCE;
        if (!atSpeed) return false;

        if (!blockerOpenCommanded) return false;
        return blockerTimer.seconds() >= BLOCKER_OPEN_DELAY_S;
    }

    public void updateShootingAuto(boolean autoShoot, double x, double y, double distance) {
        boolean inZone = isInShootingZone(x, y);

        if (!autoShoot) {
            shootState = ShootState.IDLE;
        }

        if (autoShoot && !inZone) {
            shootState = ShootState.IDLE;
        }

        switch (shootState) {
            case IDLE:
                headingLock = false;
                commandBlockerClosed();

                if (autoShoot && inZone) {
                    shootState = ShootState.ARMING;
                    break;
                }

                setFlywheelTicks(0);
                break;

            case ARMING:
                headingLock = false;
                commandBlockerClosed();

                LutTPS = computeLutTPS(distance, x, y);
                setFlywheelTicks(LutTPS);

                update();

                double currentTPS = flywheelMotorOne.getVelocity();
                if (Math.abs(targetTPS - currentTPS) <= VELOCITY_TOLERANCE) {
                    commandBlockerOpen();
                    shootState = ShootState.OPENING_BLOCKER;
                }

                if (!autoShoot || !inZone) {
                    shootState = ShootState.IDLE;
                }
                break;

            case OPENING_BLOCKER:
                headingLock = false;

                LutTPS = computeLutTPS(distance, x, y);
                setFlywheelTicks(LutTPS);

                update();
                commandBlockerOpen();

                if (flywheelReady()) {
                    shootState = ShootState.FIRING;
                }

                if (!autoShoot || !inZone) {
                    shootState = ShootState.IDLE;
                }
                break;

            case FIRING:
                headingLock = false;

                LutTPS = computeLutTPS(distance, x, y);
                setFlywheelTicks(LutTPS);

                update();
                commandBlockerOpen();

                if (!autoShoot || !inZone) {
                    shootState = ShootState.IDLE;
                }
                break;
        }
    }

    public void updateShooting(boolean shootButton, double x, double y, double distance) {
        boolean inZone = isInShootingZone(x, y);

        if (!inZone && shootState != ShootState.IDLE) {
            cancelShooting();
        }

        if (shootButton && !shootButtonLast) {
            if (shootState == ShootState.IDLE && inZone) {
                shootState = ShootState.ARMING;

                LutTPS = computeLutTPS(distance, x, y);
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
                commandBlockerClosed();
                setFlywheelTicks(0);
                break;

            case ARMING:
                headingLock = true;
                commandBlockerClosed();

                LutTPS = computeLutTPS(distance, x, y);
                setFlywheelTicks(LutTPS);

                update();

                double currentTPS = flywheelMotorOne.getVelocity();
                if (Math.abs(targetTPS - currentTPS) <= VELOCITY_TOLERANCE) {
                    commandBlockerOpen();
                    shootState = ShootState.OPENING_BLOCKER;
                }
                break;

            case OPENING_BLOCKER:
                headingLock = true;

                LutTPS = computeLutTPS(distance, x, y);
                setFlywheelTicks(LutTPS);

                update();
                commandBlockerOpen();

                if (flywheelReady()) {
                    shootState = ShootState.FIRING;
                }
                break;

            case FIRING:
                headingLock = true;

                LutTPS = computeLutTPS(distance, x, y);
                setFlywheelTicks(LutTPS);

                update();
                commandBlockerOpen();
                break;
        }
    }

    private void cancelShooting() {
        shootState = ShootState.IDLE;
        headingLock = false;
        setFlywheelTicks(0);
        commandBlockerClosed();
    }

    public boolean isHeadingLockEnabled() {
        return headingLock;
    }

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
