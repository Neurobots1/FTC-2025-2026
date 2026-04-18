package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants.ShooterHardwareConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

final class TurretAxis {

    private enum HomeState {
        SEEK_LEFT,
        BACKOFF_LEFT,
        SEEK_RIGHT,
        BACKOFF_RIGHT,
        DONE
    }

    private final DcMotorEx motor;
    private final ShooterConstants config;
    private final ElapsedTime homeTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private HomeState homeState = HomeState.SEEK_LEFT;
    private int leftStopTicks;
    private int rightStopTicks;
    private double lastPositionTicks;
    private double lastVelocityTicksPerSecond;
    private double lastRequestedAngleRadians;
    private double targetAngleRadians;
    private double lastAngleError;
    private double lastCommandPower;
    private double lastSoftLimitScale = 1.0;
    private boolean homed;

    TurretAxis(DcMotorEx motor, ShooterConstants config) {
        this.motor = motor;
        this.config = config;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(ShooterHardwareConstants.turretMotorReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        lastPositionTicks = motor.getCurrentPosition();
        loopTimer.reset();
        homeTimer.reset();
    }

    void updateHoming() {
        if (homed) {
            return;
        }

        double dt = Math.max(1e-3, loopTimer.seconds());
        loopTimer.reset();
        double position = motor.getCurrentPosition();
        lastVelocityTicksPerSecond = (position - lastPositionTicks) / dt;
        lastPositionTicks = position;

        switch (homeState) {
            case SEEK_LEFT:
                motor.setPower(-Math.abs(config.turretHomePower));
                if (stallDetected()) {
                    leftStopTicks = motor.getCurrentPosition();
                    homeState = HomeState.BACKOFF_LEFT;
                    homeTimer.reset();
                }
                break;

            case BACKOFF_LEFT:
                motor.setPower(Math.abs(config.turretBackoffPower));
                if (Math.abs(motor.getCurrentPosition() - leftStopTicks) >= config.turretBackoffTicks) {
                    homeState = HomeState.SEEK_RIGHT;
                    homeTimer.reset();
                }
                break;

            case SEEK_RIGHT:
                motor.setPower(Math.abs(config.turretHomePower));
                if (stallDetected()) {
                    rightStopTicks = motor.getCurrentPosition();
                    homeState = HomeState.BACKOFF_RIGHT;
                    homeTimer.reset();
                }
                break;

            case BACKOFF_RIGHT:
                motor.setPower(-Math.abs(config.turretBackoffPower));
                if (Math.abs(motor.getCurrentPosition() - rightStopTicks) >= config.turretBackoffTicks) {
                    homed = true;
                    homeState = HomeState.DONE;
                    targetAngleRadians = getCurrentSweepRadians();
                    lastAngleError = 0.0;
                    lastCommandPower = 0.0;
                    lastSoftLimitScale = 1.0;
                    motor.setPower(0.0);
                }
                break;

            case DONE:
                motor.setPower(0.0);
                break;
        }
    }

    void setTargetAngleRadians(double targetAngleRadians) {
        lastRequestedAngleRadians = ShooterMath.normalizeRadians(targetAngleRadians);
        double requestedSweepRadians = commandAngleToSweepRadians(targetAngleRadians);
        this.targetAngleRadians = ShooterMath.clamp(
                requestedSweepRadians,
                getMinimumCommandedSweepRadians(),
                getMaximumCommandedSweepRadians()
        );
    }

    void updateTracking() {
        if (!homed) {
            lastCommandPower = 0.0;
            lastSoftLimitScale = 1.0;
            return;
        }
        double currentSweepRadians = getCurrentSweepRadians();
        double angleError = currentSweepRadians - targetAngleRadians;
        double derivative = angleError - lastAngleError;
        lastAngleError = angleError;
        double power = config.turretPositionKp * angleError
                + config.turretPositionKd * derivative
                + Math.signum(angleError) * config.turretKf;
        if (Math.abs(angleError) < Math.toRadians(config.turretControlToleranceDeg)) {
            power = 0.0;
        }
        power = applySoftLimits(power, currentSweepRadians);
        lastCommandPower = power;
        motor.setPower(power);
    }

    boolean isHomed() {
        return homed;
    }

    boolean atTarget() {
        return atTarget(Math.toRadians(config.turretShotReadyToleranceDeg));
    }

    boolean atTarget(double toleranceRadians) {
        return homed && Math.abs(targetAngleRadians - getCurrentSweepRadians()) <= toleranceRadians;
    }

    double getCurrentAngleRadians() {
        if (!homed && rightStopTicks == leftStopTicks) {
            return 0.0;
        }
        return ShooterMath.normalizeRadians(getCurrentSweepRadians());
    }

    double getTargetAngleRadians() {
        return ShooterMath.normalizeRadians(targetAngleRadians);
    }

    double getLastAngleErrorRadians() {
        return lastAngleError;
    }

    double getLastCommandPower() {
        return lastCommandPower;
    }

    double getLastSoftLimitScale() {
        return lastSoftLimitScale;
    }

    double getMotorCurrentAmps() {
        try {
            return motor.getCurrent(CurrentUnit.AMPS);
        } catch (Exception ignored) {
            return 0.0;
        }
    }

    double getLastVelocityTicksPerSecond() {
        return lastVelocityTicksPerSecond;
    }

    String getHomeStateName() {
        return homeState.name();
    }

    int getLeftStopTicks() {
        return leftStopTicks;
    }

    int getRightStopTicks() {
        return rightStopTicks;
    }

    boolean loadHomeCalibration(int leftStopTicks, int rightStopTicks) {
        if (rightStopTicks <= leftStopTicks) {
            return false;
        }

        this.leftStopTicks = leftStopTicks;
        this.rightStopTicks = rightStopTicks;
        homeState = HomeState.DONE;
        homed = true;
        lastPositionTicks = motor.getCurrentPosition();
        lastVelocityTicksPerSecond = 0.0;
        lastAngleError = 0.0;
        lastCommandPower = 0.0;
        lastSoftLimitScale = 1.0;
        targetAngleRadians = ShooterMath.clamp(
                getCurrentSweepRadians(),
                getMinimumCommandedSweepRadians(),
                getMaximumCommandedSweepRadians()
        );
        loopTimer.reset();
        homeTimer.reset();
        motor.setPower(0.0);
        return true;
    }

    void restartHoming() {
        homed = false;
        homeState = HomeState.SEEK_LEFT;
        leftStopTicks = 0;
        rightStopTicks = 0;
        targetAngleRadians = 0.0;
        lastAngleError = 0.0;
        lastCommandPower = 0.0;
        lastSoftLimitScale = 1.0;
        lastPositionTicks = motor.getCurrentPosition();
        lastVelocityTicksPerSecond = 0.0;
        loopTimer.reset();
        homeTimer.reset();
        motor.setPower(0.0);
    }

    double getMinimumCommandedAngleRadians() {
        return getMinimumCommandedSweepRadians();
    }

    double getMaximumCommandedAngleRadians() {
        return getMaximumCommandedSweepRadians();
    }

    boolean isRequestedAngleInForwardDeadZone() {
        double halfForbiddenRadians = getHalfForbiddenRadians();
        if (!homed || halfForbiddenRadians <= 1e-6) {
            return false;
        }

        double localRequested = ShooterMath.normalizeRadians(
                lastRequestedAngleRadians - getForwardDeadZoneCenterRadians()
        );
        return Math.abs(localRequested) < halfForbiddenRadians;
    }

    double getDeadZoneEscapeChassisTurnDirection() {
        if (!isRequestedAngleInForwardDeadZone()) {
            return 0.0;
        }

        double localRequested = ShooterMath.normalizeRadians(
                lastRequestedAngleRadians - getForwardDeadZoneCenterRadians()
        );
        if (Math.abs(localRequested) > 1e-6) {
            return -Math.signum(localRequested);
        }

        double localCurrent = ShooterMath.normalizeRadians(
                getCurrentAngleRadians() - getForwardDeadZoneCenterRadians()
        );
        if (Math.abs(localCurrent) > 1e-6) {
            return -Math.signum(localCurrent);
        }

        double localTarget = ShooterMath.normalizeRadians(
                getTargetAngleRadians() - getForwardDeadZoneCenterRadians()
        );
        if (Math.abs(localTarget) > 1e-6) {
            return -Math.signum(localTarget);
        }

        return 1.0;
    }

    private double applySoftLimits(double power, double currentSweepRadians) {
        if (!homed || power == 0.0) {
            lastSoftLimitScale = 1.0;
            return power;
        }

        double slowZoneRadians = Math.toRadians(Math.max(0.0, config.turretLimitSlowZoneDeg));
        double minScale = ShooterMath.clamp(config.turretLimitMinScale, 0.0, 1.0);
        double scale = 1.0;

        if (power < 0.0) {
            double distanceToLimit = getMaximumCommandedSweepRadians() - currentSweepRadians;
            if (distanceToLimit <= 0.0) {
                lastSoftLimitScale = 0.0;
                return 0.0;
            }
            if (slowZoneRadians > 1e-6 && distanceToLimit < slowZoneRadians) {
                scale = minScale + (1.0 - minScale) * (distanceToLimit / slowZoneRadians);
            }
        } else {
            double distanceToLimit = currentSweepRadians - getMinimumCommandedSweepRadians();
            if (distanceToLimit <= 0.0) {
                lastSoftLimitScale = 0.0;
                return 0.0;
            }
            if (slowZoneRadians > 1e-6 && distanceToLimit < slowZoneRadians) {
                scale = minScale + (1.0 - minScale) * (distanceToLimit / slowZoneRadians);
            }
        }

        lastSoftLimitScale = ShooterMath.clamp(scale, 0.0, 1.0);
        return power * lastSoftLimitScale;
    }

    private double getCurrentSweepRadians() {
        double sweepTicks = Math.max(1.0, rightStopTicks - leftStopTicks);
        double travelFraction = (motor.getCurrentPosition() - leftStopTicks) / sweepTicks;
        travelFraction = ShooterMath.clamp(travelFraction, 0.0, 1.0);
        return getLeftStopSweepRadians() - travelFraction * getMechanicalTravelRadians();
    }

    private double commandAngleToSweepRadians(double desiredAngleRadians) {
        double localDesired = ShooterMath.normalizeRadians(desiredAngleRadians - getForwardDeadZoneCenterRadians());
        double halfForbiddenRadians = getHalfForbiddenRadians();

        if (localDesired <= -halfForbiddenRadians) {
            return getForwardDeadZoneCenterRadians() + localDesired;
        }
        if (localDesired >= halfForbiddenRadians) {
            return getForwardDeadZoneCenterRadians() + localDesired - 2.0 * Math.PI;
        }

        double currentSweepRadians = homed
                ? getCurrentSweepRadians()
                : getLeftStopSweepRadians();
        double leftEdgeRadians = getLeftStopSweepRadians();
        double rightEdgeRadians = getRightStopSweepRadians();
        return Math.abs(currentSweepRadians - leftEdgeRadians) <= Math.abs(currentSweepRadians - rightEdgeRadians)
                ? leftEdgeRadians
                : rightEdgeRadians;
    }

    private double getMechanicalTravelRadians() {
        return Math.toRadians(ShooterHardwareConstants.turretMechanicalRangeDeg);
    }

    private double getHalfForbiddenRadians() {
        return 0.5 * Math.toRadians(Math.max(0.0, ShooterHardwareConstants.turretForbiddenWidthDeg));
    }

    private double getForwardDeadZoneCenterRadians() {
        return Math.toRadians(
                ShooterHardwareConstants.turretCenterOffsetDeg
                        + ShooterHardwareConstants.turretForbiddenCenterDeg
        );
    }

    private double getLeftStopSweepRadians() {
        return getForwardDeadZoneCenterRadians() - getHalfForbiddenRadians();
    }

    private double getRightStopSweepRadians() {
        return getLeftStopSweepRadians() - getMechanicalTravelRadians();
    }

    private double getMinimumCommandedSweepRadians() {
        return getRightStopSweepRadians() + Math.toRadians(config.turretLimitBufferDeg);
    }

    private double getMaximumCommandedSweepRadians() {
        return getLeftStopSweepRadians() - Math.toRadians(config.turretLimitBufferDeg);
    }

    private boolean stallDetected() {
        double current = 0.0;
        try {
            current = motor.getCurrent(CurrentUnit.AMPS);
        } catch (Exception ignored) {
        }

        boolean velocityLow = Math.abs(lastVelocityTicksPerSecond) <= config.turretHomeVelocityTicksPerSecond;
        boolean currentHigh = current >= config.turretHomeCurrentAmps;
        if (velocityLow && currentHigh) {
            if (homeTimer.seconds() >= config.turretHomeHoldSeconds) {
                return true;
            }
        } else {
            homeTimer.reset();
        }
        return false;
    }
}
