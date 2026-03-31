package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "TUNE_FLYWHEEL_AUTO", group = "Tuning")
public class PrecisionFlywheelAutoTunerOpMode extends OpMode {

    private static final class KvSample {
        final double normalizedPower;
        final double rpm;

        KvSample(double normalizedPower, double rpm) {
            this.normalizedPower = normalizedPower;
            this.rpm = rpm;
        }
    }

    private static final class StepSample {
        final double timeSeconds;
        final double rpm;

        StepSample(double timeSeconds, double rpm) {
            this.timeSeconds = timeSeconds;
            this.rpm = rpm;
        }
    }

    private enum State {
        IDLE,
        KS_RAMP,
        KV_SWEEP,
        STEP_TEST,
        DONE
    }

    private static final String LOG_TAG = "FlywheelAutoTuner";
    private static final double KS_SPINUP_RPM = 150.0;

    private final ShooterConstants config = new ShooterConstants();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final List<KvSample> kvSamples = new ArrayList<>();
    private final List<StepSample> stepResponseSamples = new ArrayList<>();

    private DcMotorEx left;
    private DcMotorEx right;
    private VoltageSensor voltageSensor;
    private State state = State.IDLE;
    private int kvIndex;
    private double ks;
    private double kv;
    private double plantGain;
    private double timeConstant;
    private double baselineRpm;
    private double targetRpm63;
    private FlywheelVelocityController.Gains suggestedGains;
    private boolean prevX;
    private double lastLoopTime;
    private double lastRpm;
    private double stableTime;
    private double lastMeasuredRate;
    private double stableRpmAccumulator;
    private int stableRpmSamples;

    private final double[] sweepPowers = {0.28, 0.36, 0.44, 0.52, 0.60, 0.68};

    @Override
    public void init() {
        left = hardwareMap.get(DcMotorEx.class, config.leftFlywheelName);
        right = hardwareMap.get(DcMotorEx.class, config.rightFlywheelName);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        left.setDirection(config.leftFlywheelReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        right.setDirection(config.rightFlywheelReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        if (gamepad1.x && !prevX && state == State.IDLE) {
            state = State.KS_RAMP;
            stateTimer.reset();
            kvSamples.clear();
            kvIndex = 0;
            ks = 0.0;
            kv = 0.0;
            plantGain = 0.0;
            timeConstant = 0.0;
            baselineRpm = Double.NaN;
            targetRpm63 = 0.0;
            suggestedGains = null;
            stepResponseSamples.clear();
            resetStabilityTracker();
        }
        prevX = gamepad1.x;

        updateStabilityTracker();

        switch (state) {
            case IDLE:
                setPower(0.0);
                break;

            case KS_RAMP:
                double rampPower = ShooterMath.clamp(0.06 + stateTimer.seconds() * 0.05, 0.0, 0.35);
                setPower(rampPower);
                if (getRpm() >= KS_SPINUP_RPM) {
                    ks = normalizePowerToNominalVoltage(rampPower);
                    state = State.KV_SWEEP;
                    stateTimer.reset();
                    resetStabilityTracker();
                    setPower(0.0);
                }
                break;

            case KV_SWEEP:
                if (kvIndex >= sweepPowers.length) {
                    kv = fitKv();
                    state = State.STEP_TEST;
                    stateTimer.reset();
                    resetStabilityTracker();
                    setPower(0.0);
                    break;
                }

                double power = sweepPowers[kvIndex];
                setPower(power);
                if (sampleWindowComplete(config.flywheelAutoTuneSweepMinSeconds,
                        config.flywheelAutoTuneSweepMaxSeconds)) {
                    kvSamples.add(new KvSample(
                            normalizePowerToNominalVoltage(power),
                            getStableRpmEstimate()
                    ));
                    kvIndex++;
                    stateTimer.reset();
                    resetStabilityTracker();
                }
                break;

            case STEP_TEST:
                double stepPower = config.flywheelAutoTuneStepPower;
                if (Double.isNaN(baselineRpm)) {
                    setPower(0.0);
                    if (currentRpmBelow(config.flywheelAutoTuneStoppedRpm)
                            || stateTimer.seconds() >= config.flywheelAutoTuneStopTimeoutSeconds) {
                        baselineRpm = getRpm();
                        stepResponseSamples.clear();
                        stateTimer.reset();
                        resetStabilityTracker();
                    }
                } else {
                    setPower(stepPower);
                    stepResponseSamples.add(new StepSample(stateTimer.seconds(), getRpm()));
                    if (sampleWindowComplete(config.flywheelAutoTuneStepMinSeconds,
                            config.flywheelAutoTuneStepMaxSeconds)) {
                        double normalizedStepPower = normalizePowerToNominalVoltage(stepPower);
                        double finalRpm = getStableRpmEstimate();
                        plantGain = (finalRpm - baselineRpm) / Math.max(1e-6, normalizedStepPower);
                        targetRpm63 = baselineRpm + 0.632 * (finalRpm - baselineRpm);
                        timeConstant = computeTimeConstant(targetRpm63, config.flywheelAutoTuneStepMaxSeconds);
                        suggestedGains = synthesizeGains(ks, kv, plantGain, timeConstant);
                        state = State.DONE;
                        stateTimer.reset();
                        RobotLog.ii(LOG_TAG, String.format(Locale.US,
                                "Suggested gains: kP=%.6f kI=%.6f kD=%.6f kV=%.6f kS=%.6f",
                                suggestedGains.kP,
                                suggestedGains.kI,
                                suggestedGains.kD,
                                suggestedGains.kV,
                                suggestedGains.kS));
                    }
                }
                break;

            case DONE:
                setPower(0.0);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("RPM", "%.1f", getRpm());
        telemetry.addData("Voltage", "%.2f", voltageSensor.getVoltage());
        telemetry.addData("kS", "%.5f", ks);
        telemetry.addData("kV", "%.6f", kv);
        telemetry.addData("Plant Gain", "%.3f", plantGain);
        telemetry.addData("Tau", "%.3f", timeConstant);
        telemetry.addData("RPM rate", "%.1f rpm/s", lastMeasuredRate);
        telemetry.addData("Stable for", "%.2f s", stableTime);
        if (suggestedGains != null) {
            telemetry.addData("kP", "%.6f", suggestedGains.kP);
            telemetry.addData("kI", "%.6f", suggestedGains.kI);
            telemetry.addData("kD", "%.6f", suggestedGains.kD);
            telemetry.addData("kV ff", "%.6f", suggestedGains.kV);
            telemetry.addData("kS ff", "%.6f", suggestedGains.kS);
            telemetry.addLine("Suggested gains were logged to RobotLog.");
        } else {
            telemetry.addLine("Press X to run the auto-tune sweep.");
        }
        telemetry.update();
    }

    private FlywheelVelocityController.Gains synthesizeGains(double kS, double kV, double plantGain, double tau) {
        double lambda = Math.max(0.25, tau * 1.5);
        double effectiveGain = Math.max(1e-3, plantGain);
        double kP = tau / (effectiveGain * lambda);
        double kI = kP / Math.max(tau, 1e-3);
        double kD = 0.0;
        return new FlywheelVelocityController.Gains(kP, kI, kD, kV, kS);
    }

    private double fitKv() {
        double sumRpm = 0.0;
        double sumPower = 0.0;
        double sumRpmSq = 0.0;
        double sumRpmPower = 0.0;
        int count = 0;
        for (KvSample sample : kvSamples) {
            if (sample.rpm > 50.0 && sample.normalizedPower > ks) {
                sumRpm += sample.rpm;
                sumPower += sample.normalizedPower;
                sumRpmSq += sample.rpm * sample.rpm;
                sumRpmPower += sample.rpm * sample.normalizedPower;
                count++;
            }
        }

        if (count < 2) {
            return 0.0002;
        }

        double denominator = count * sumRpmSq - sumRpm * sumRpm;
        if (Math.abs(denominator) < 1e-6) {
            return 0.0002;
        }

        double fittedKv = (count * sumRpmPower - sumRpm * sumPower) / denominator;
        double fittedKs = (sumPower - fittedKv * sumRpm) / count;
        ks = ShooterMath.clamp(fittedKs, 0.0, 1.0);
        return Math.max(1e-6, fittedKv);
    }

    private double getRpm() {
        DcMotorEx feedbackMotor = config.flywheelFeedbackMotor == ShooterConstants.FlywheelFeedbackMotor.RIGHT
                ? right
                : left;
        double measuredRpm = FlywheelVelocityController.ticksPerSecondToRpm(feedbackMotor.getVelocity());
        return config.flywheelFeedbackEncoderReversed ? -measuredRpm : measuredRpm;
    }

    private void updateStabilityTracker() {
        double elapsed = stateTimer.seconds();
        double rpm = getRpm();
        if (lastLoopTime == 0.0) {
            lastLoopTime = elapsed;
            lastRpm = rpm;
            lastMeasuredRate = 0.0;
            stableTime = 0.0;
            return;
        }

        double dt = Math.max(1e-3, elapsed - lastLoopTime);
        lastMeasuredRate = (rpm - lastRpm) / dt;
        if (Math.abs(lastMeasuredRate) <= config.flywheelAutoTuneStableRpmRate) {
            stableTime += dt;
            stableRpmAccumulator += rpm;
            stableRpmSamples++;
        } else {
            stableTime = 0.0;
            stableRpmAccumulator = 0.0;
            stableRpmSamples = 0;
        }

        lastLoopTime = elapsed;
        lastRpm = rpm;
    }

    private void resetStabilityTracker() {
        lastLoopTime = 0.0;
        lastRpm = getRpm();
        stableTime = 0.0;
        lastMeasuredRate = 0.0;
        stableRpmAccumulator = 0.0;
        stableRpmSamples = 0;
    }

    private boolean sampleWindowComplete(double minSeconds, double maxSeconds) {
        return stateTimer.seconds() >= maxSeconds
                || (stateTimer.seconds() >= minSeconds
                && stableTime >= config.flywheelAutoTuneStableWindowSeconds);
    }

    private boolean currentRpmBelow(double rpm) {
        return Math.abs(getRpm()) <= rpm;
    }

    private double computeTimeConstant(double rpm63Target, double fallbackSeconds) {
        for (StepSample sample : stepResponseSamples) {
            if (sample.rpm >= rpm63Target) {
                return sample.timeSeconds;
            }
        }
        return fallbackSeconds;
    }

    private double getStableRpmEstimate() {
        if (stableRpmSamples > 0) {
            return stableRpmAccumulator / stableRpmSamples;
        }
        return getRpm();
    }

    private double normalizePowerToNominalVoltage(double rawPower) {
        return rawPower * (voltageSensor.getVoltage() / Math.max(1e-6, config.nominalBatteryVoltage));
    }

    private void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }
}
