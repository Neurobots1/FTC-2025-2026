package org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Flywheel Auto Tuner", group = "Shooter")
public class PrecisionFlywheelAutoTunerOpMode extends OpMode {

    private enum State {
        IDLE,
        KS_RAMP,
        KV_SWEEP,
        STEP_TEST,
        DONE
    }

    private static final String LOG_TAG = "FlywheelAutoTuner";

    private final PrecisionShooterConfig config = new PrecisionShooterConfig();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final List<double[]> kvSamples = new ArrayList<>();

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
            suggestedGains = null;
        }
        prevX = gamepad1.x;

        switch (state) {
            case IDLE:
                setPower(0.0);
                break;

            case KS_RAMP:
                double rampPower = ShooterMath.clamp(0.06 + stateTimer.seconds() * 0.05, 0.0, 0.35);
                setPower(rampPower);
                if (getRpm() >= 150.0) {
                    ks = rampPower;
                    state = State.KV_SWEEP;
                    stateTimer.reset();
                    setPower(0.0);
                }
                break;

            case KV_SWEEP:
                if (kvIndex >= sweepPowers.length) {
                    kv = fitKv();
                    baselineRpm = 0.0;
                    state = State.STEP_TEST;
                    stateTimer.reset();
                    setPower(0.0);
                    break;
                }

                double power = sweepPowers[kvIndex];
                setPower(power);
                if (stateTimer.seconds() >= 1.3) {
                    kvSamples.add(new double[]{power, getRpm()});
                    kvIndex++;
                    stateTimer.reset();
                    setPower(0.0);
                }
                break;

            case STEP_TEST:
                double stepPower = 0.62;
                if (stateTimer.seconds() < 0.25) {
                    baselineRpm = getRpm();
                    setPower(0.0);
                } else {
                    setPower(stepPower);
                    double steadyRpmGuess = Math.max(1.0, (stepPower - ks) / Math.max(1e-6, kv));
                    targetRpm63 = baselineRpm + 0.632 * (steadyRpmGuess - baselineRpm);
                    if (timeConstant == 0.0 && getRpm() >= targetRpm63) {
                        timeConstant = stateTimer.seconds() - 0.25;
                    }
                    if (stateTimer.seconds() >= 2.0) {
                        double finalRpm = getRpm();
                        plantGain = (finalRpm - baselineRpm) / stepPower;
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
        double lambda = Math.max(0.08, tau * 0.75);
        double effectiveGain = Math.max(1e-3, plantGain);
        double kP = tau / (effectiveGain * lambda);
        double kI = kP / Math.max(lambda, 1e-3);
        double kD = 0.0;
        return new FlywheelVelocityController.Gains(kP, kI, kD, kV, kS);
    }

    private double fitKv() {
        double total = 0.0;
        int count = 0;
        for (double[] sample : kvSamples) {
            double power = sample[0];
            double rpm = sample[1];
            if (rpm > 50.0 && power > ks) {
                total += (power - ks) / rpm;
                count++;
            }
        }
        return count == 0 ? 0.0002 : total / count;
    }

    private double getRpm() {
        return 0.5 * (
                FlywheelVelocityController.ticksPerSecondToRpm(left.getVelocity())
                        + FlywheelVelocityController.ticksPerSecondToRpm(right.getVelocity())
        );
    }

    private void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }
}
