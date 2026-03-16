package org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

final class TurretAxis {

    private enum HomeState {
        SEEK_LEFT,
        BACKOFF_LEFT,
        SEEK_RIGHT,
        BACKOFF_RIGHT,
        DONE
    }

    private final DcMotorEx motor;
    private final PrecisionShooterConfig config;
    private final ElapsedTime homeTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private HomeState homeState = HomeState.SEEK_LEFT;
    private int leftStopTicks;
    private int rightStopTicks;
    private double lastPositionTicks;
    private double lastVelocityTicksPerSecond;
    private double targetAngleRadians;
    private double lastAngleError;
    private boolean homed;

    TurretAxis(DcMotorEx motor, PrecisionShooterConfig config) {
        this.motor = motor;
        this.config = config;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(config.turretMotorReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
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
                    targetAngleRadians = getCurrentAngleRadians();
                    motor.setPower(0.0);
                }
                break;

            case DONE:
                motor.setPower(0.0);
                break;
        }
    }

    void setTargetAngleRadians(double targetAngleRadians) {
        double halfRange = Math.toRadians(config.turretMechanicalRangeDeg) * 0.5;
        double offset = Math.toRadians(config.turretCenterOffsetDeg);
        this.targetAngleRadians = ShooterMath.clamp(targetAngleRadians, -halfRange + offset, halfRange + offset);
    }

    void updateTracking() {
        if (!homed) {
            return;
        }
        double angleError = ShooterMath.normalizeRadians(targetAngleRadians - getCurrentAngleRadians());
        double derivative = angleError - lastAngleError;
        lastAngleError = angleError;
        double power = config.turretPositionKp * angleError
                + config.turretPositionKd * derivative
                + Math.signum(angleError) * config.turretStatic;
        power = ShooterMath.clamp(power, -config.turretMaxCommand, config.turretMaxCommand);
        if (Math.abs(angleError) < Math.toRadians(config.turretAngleToleranceDeg)) {
            power = 0.0;
        }
        motor.setPower(power);
    }

    boolean isHomed() {
        return homed;
    }

    boolean atTarget() {
        return homed && Math.abs(ShooterMath.normalizeRadians(targetAngleRadians - getCurrentAngleRadians()))
                <= Math.toRadians(config.turretAngleToleranceDeg);
    }

    double getCurrentAngleRadians() {
        if (!homed && rightStopTicks == leftStopTicks) {
            return 0.0;
        }
        double centerTicks = 0.5 * (leftStopTicks + rightStopTicks);
        double sweepTicks = Math.max(1.0, rightStopTicks - leftStopTicks);
        double normalized = (motor.getCurrentPosition() - centerTicks) / sweepTicks;
        return normalized * Math.toRadians(config.turretMechanicalRangeDeg)
                + Math.toRadians(config.turretCenterOffsetDeg);
    }

    double getTargetAngleRadians() {
        return targetAngleRadians;
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
