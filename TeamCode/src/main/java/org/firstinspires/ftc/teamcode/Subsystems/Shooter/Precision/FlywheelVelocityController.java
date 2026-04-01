package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

final class FlywheelVelocityController {

    static final class Gains {
        final double kP;
        final double kI;
        final double kD;
        final double kV;
        final double kS;

        Gains(double kP, double kI, double kD, double kV, double kS) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kV = kV;
            this.kS = kS;
        }
    }

    private final DcMotorEx left;
    private final DcMotorEx right;
    private final VoltageSensor voltageSensor;
    private final ShooterConstants config;
    private final ElapsedTime timer = new ElapsedTime();

    private Gains gainsOverride;
    private double targetRpm;
    private double integral;
    private double lastError;
    private double measuredRpm;
    private double filteredMeasuredRpm;
    private boolean filteredMeasuredRpmInitialized;

    FlywheelVelocityController(DcMotorEx left,
                               DcMotorEx right,
                               VoltageSensor voltageSensor,
                               ShooterConstants config) {
        this.left = left;
        this.right = right;
        this.voltageSensor = voltageSensor;
        this.config = config;

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setDirection(config.leftFlywheelReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        right.setDirection(config.rightFlywheelReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        timer.reset();
    }

    void setGains(Gains gains) {
        this.gainsOverride = gains;
    }

    Gains getGains() {
        return currentGains();
    }

    void setTargetRpm(double targetRpm) {
        this.targetRpm = Math.max(0.0, targetRpm);
    }

    double getTargetRpm() {
        return targetRpm;
    }

    double getMeasuredRpm() {
        measuredRpm = readMeasuredRpm();
        return measuredRpm;
    }

    double getFilteredMeasuredRpm() {
        if (!filteredMeasuredRpmInitialized) {
            filteredMeasuredRpm = getMeasuredRpm();
            filteredMeasuredRpmInitialized = true;
        }
        return filteredMeasuredRpmInitialized ? filteredMeasuredRpm : measuredRpm;
    }

    void stop() {
        targetRpm = 0.0;
        integral = 0.0;
        lastError = 0.0;
        measuredRpm = readMeasuredRpm();
        filteredMeasuredRpmInitialized = false;
        left.setPower(0.0);
        right.setPower(0.0);
    }

    void update(double maxRpm, double nominalVoltage, double integralLimit) {
        double dt = Math.max(1e-3, timer.seconds());
        timer.reset();

        if (targetRpm <= 1.0) {
            stop();
            return;
        }

        measuredRpm = getMeasuredRpm();
        updateFilteredMeasuredRpm();
        double error = targetRpm - measuredRpm;
        integral += error * dt;
        integral = ShooterMath.clamp(integral, -integralLimit, integralLimit);
        double derivative = (error - lastError) / dt;
        lastError = error;

        Gains gains = currentGains();
        double voltageScale = nominalVoltage / Math.max(1e-6, voltageSensor.getVoltage());
        double feedforward = gains.kV * targetRpm + gains.kS;
        double feedback = gains.kP * error + gains.kI * integral + gains.kD * derivative;
        double output = ShooterMath.clamp((feedforward + feedback) * voltageScale, 0.0, 1.0);

        left.setPower(output);
        right.setPower(output);
    }

    boolean atSpeed(double toleranceRpm) {
        return Math.abs(targetRpm - getMeasuredRpm()) <= toleranceRpm;
    }

    static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / ShooterConstants.TICKS_PER_REV;
    }

    private DcMotorEx feedbackMotor() {
        return config.flywheelFeedbackMotor == ShooterConstants.FlywheelFeedbackMotor.RIGHT
                ? right
                : left;
    }

    private double readMeasuredRpm() {
        // The flywheels are mechanically linked, so we only need one encoder feedback source.
        double rawMeasuredRpm = ticksPerSecondToRpm(feedbackMotor().getVelocity());
        return config.flywheelFeedbackEncoderReversed ? -rawMeasuredRpm : rawMeasuredRpm;
    }

    private void updateFilteredMeasuredRpm() {
        if (!filteredMeasuredRpmInitialized) {
            filteredMeasuredRpm = measuredRpm;
            filteredMeasuredRpmInitialized = true;
            return;
        }

        filteredMeasuredRpm += (measuredRpm - filteredMeasuredRpm) * config.flywheelCompensationFilterGain;
    }

    private Gains currentGains() {
        if (gainsOverride != null) {
            return gainsOverride;
        }
        return new Gains(
                config.flywheelKp,
                config.flywheelKi,
                config.flywheelKd,
                config.flywheelKv,
                config.flywheelKs
        );
    }
}
