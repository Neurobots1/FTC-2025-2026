package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tuning;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.PedroConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterHardwareConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TeleopConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;

@TeleOp(name = "TUNE_TURRET", group = "Tuning")
public class TurretTuningTeleOp extends OpMode {

    private static final double TRIGGER_SLEW_RATE_DEG_PER_SECOND = 90.0;
    private static final double DPAD_STEP_DEG = 5.0;
    private static final double BUMPER_STEP_DEG = 15.0;
    private static final double AUTO_SWEEP_MARGIN_DEG = 6.0;
    private static final double AUTO_SWEEP_SPEED_DEG_PER_SECOND = 75.0;
    private static final double JUMP_TEST_MARGIN_DEG = 20.0;

    private final ShooterConstants config = new ShooterConstants();
    private final ElapsedTime dpadTimer = new ElapsedTime();

    private Follower follower;
    private PrecisionShooterSubsystem shooter;
    private double manualTargetDeg;
    private boolean autoSweepEnabled;
    private int autoSweepDirection = 1;
    private long lastLoopNs;
    private boolean prevA;
    private boolean prevB;
    private boolean prevX;
    private boolean prevLeftBumper;
    private boolean prevRightBumper;
    private boolean jumpTargetHigh;

    @Override
    public void init() {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(TeleopConstants.DRIVER_START_POSE);
        follower.setPose(TeleopConstants.DRIVER_START_POSE);
        follower.update();

        shooter = PrecisionShooterSubsystem.create(hardwareMap, follower, config);
        shooter.setAutoAimEnabled(false);
        shooter.setSpinEnabled(false);
        shooter.requestFire(false);

        manualTargetDeg = 0.0;
        autoSweepEnabled = false;
        autoSweepDirection = 1;
        jumpTargetHigh = false;
        dpadTimer.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(TeleopConstants.DRIVER_START_POSE);
        follower.setPose(TeleopConstants.DRIVER_START_POSE);
        shooter.start();
        shooter.setAutoAimEnabled(false);
        shooter.setSpinEnabled(false);
        shooter.requestFire(false);
        shooter.setManualTurretAngleOverrideDegrees(manualTargetDeg);
        lastLoopNs = System.nanoTime();
    }

    @Override
    public void loop() {
        follower.update();

        double dtSeconds = getLoopDtSeconds();

        if (!shooter.isTurretEnabled()) {
            telemetry.addLine("Turret is disabled in ShooterHardwareConstants.");
            telemetry.addLine("Set turretEnabled = true and make sure the motor name is correct.");
            telemetry.update();
            return;
        }

        handleButtons();

        if (shooter.isTurretHomed()) {
            updateManualTarget(dtSeconds);
            manualTargetDeg = clamp(
                    manualTargetDeg,
                    shooter.getTurretMinTargetDegrees(),
                    shooter.getTurretMaxTargetDegrees()
            );
        } else {
            manualTargetDeg = 0.0;
            autoSweepEnabled = false;
        }

        shooter.setAutoAimEnabled(false);
        shooter.setSpinEnabled(false);
        shooter.requestFire(false);
        shooter.setManualTurretAngleOverrideDegrees(manualTargetDeg);
        shooter.update();

        telemetry.addData("Turret Enabled", shooter.isTurretEnabled());
        telemetry.addData("Shoot On Move", ShooterConstants.shootOnMoveEnabled);
        telemetry.addData("Homed", shooter.isTurretHomed());
        telemetry.addData("Auto Sweep", autoSweepEnabled);
        telemetry.addData("Home State", shooter.getTurretHomeState());
        telemetry.addData("Target Deg", "%.2f", manualTargetDeg);
        telemetry.addData("Current Deg", "%.2f", shooter.getTurretAngleDegrees());
        telemetry.addData("Error Deg", "%.2f", shooter.getTurretErrorDegrees());
        telemetry.addData("Command", "%.3f", shooter.getTurretCommandPower());
        telemetry.addData("Soft Limit Scale", "%.2f", shooter.getTurretSoftLimitScale());
        telemetry.addData("Turret Current A", "%.2f", shooter.getTurretCurrentAmps());
        telemetry.addData("Turret Vel Ticks/S", "%.2f", shooter.getTurretVelocityTicksPerSecond());
        telemetry.addData("Left Stop Ticks", shooter.getTurretLeftStopTicks());
        telemetry.addData("Right Stop Ticks", shooter.getTurretRightStopTicks());
        telemetry.addData("Range Ticks", "%.0f", ShooterHardwareConstants.turretMechanicalRangeTicks);
        telemetry.addData("Safe Range", "%.2f to %.2f", shooter.getTurretMinTargetDegrees(), shooter.getTurretMaxTargetDegrees());
        telemetry.addData("Kp", "%.3f", ShooterConstants.turretPositionKp);
        telemetry.addData("Kd", "%.3f", ShooterConstants.turretPositionKd);
        telemetry.addData("Kf", "%.3f", ShooterConstants.turretKf);
        telemetry.addData("Control Tol Deg", "%.2f", ShooterConstants.turretControlToleranceDeg);
        telemetry.addData("Shot Ready Tol Deg", "%.2f", ShooterConstants.turretShotReadyToleranceDeg);
        telemetry.addData("Limit Buffer Deg", "%.2f", ShooterConstants.turretLimitBufferDeg);
        telemetry.addData("Slow Zone Deg", "%.2f", ShooterConstants.turretLimitSlowZoneDeg);
        telemetry.addData("Limit Min Scale", "%.2f", ShooterConstants.turretLimitMinScale);
        telemetry.addData("Forbidden Center Deg", "%.2f", ShooterHardwareConstants.turretForbiddenCenterDeg);
        telemetry.addData("Forbidden Width Deg", "%.2f", ShooterHardwareConstants.turretForbiddenWidthDeg);
        telemetry.addData("Home Current Threshold A", "%.2f", ShooterConstants.turretHomeCurrentAmps);
        telemetry.addData("Home Velocity Threshold", "%.2f", ShooterConstants.turretHomeVelocityTicksPerSecond);
        telemetry.addLine("Wait for homing first. Then tune Kp up, add Kd to damp, add Kf only until it starts moving cleanly.");
        telemetry.addLine("Triggers slew target, Dpad left/right = 5 deg, LB/RB = 15 deg, B jump test, A center, X sweep on/off.");
        telemetry.update();
    }

    private void handleButtons() {
        if (gamepad1.a && !prevA) {
            manualTargetDeg = 0.0;
            autoSweepEnabled = false;
        }

        if (gamepad1.b && !prevB && shooter.isTurretHomed()) {
            manualTargetDeg = getJumpTargetDegrees(!jumpTargetHigh);
            jumpTargetHigh = !jumpTargetHigh;
            autoSweepEnabled = false;
        }

        if (gamepad1.x && !prevX && shooter.isTurretHomed()) {
            autoSweepEnabled = !autoSweepEnabled;
            autoSweepDirection = 1;
        }

        if (gamepad1.right_bumper && !prevRightBumper && shooter.isTurretHomed()) {
            manualTargetDeg += BUMPER_STEP_DEG;
            autoSweepEnabled = false;
        }

        if (gamepad1.left_bumper && !prevLeftBumper && shooter.isTurretHomed()) {
            manualTargetDeg -= BUMPER_STEP_DEG;
            autoSweepEnabled = false;
        }

        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevLeftBumper = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;
    }

    private void updateManualTarget(double dtSeconds) {
        if (autoSweepEnabled) {
            double minTarget = shooter.getTurretMinTargetDegrees() + AUTO_SWEEP_MARGIN_DEG;
            double maxTarget = shooter.getTurretMaxTargetDegrees() - AUTO_SWEEP_MARGIN_DEG;
            if (minTarget > maxTarget) {
                double midpoint = 0.5 * (minTarget + maxTarget);
                minTarget = midpoint;
                maxTarget = midpoint;
            }

            manualTargetDeg += autoSweepDirection * AUTO_SWEEP_SPEED_DEG_PER_SECOND * dtSeconds;
            if (manualTargetDeg >= maxTarget) {
                manualTargetDeg = maxTarget;
                autoSweepDirection = -1;
            } else if (manualTargetDeg <= minTarget) {
                manualTargetDeg = minTarget;
                autoSweepDirection = 1;
            }
            return;
        }

        double triggerInput = gamepad1.right_trigger - gamepad1.left_trigger;
        if (Math.abs(triggerInput) > 0.02) {
            manualTargetDeg += triggerInput * TRIGGER_SLEW_RATE_DEG_PER_SECOND * dtSeconds;
        }

        if (dpadTimer.milliseconds() >= 120.0) {
            if (gamepad1.dpad_right) {
                manualTargetDeg += DPAD_STEP_DEG;
                dpadTimer.reset();
            } else if (gamepad1.dpad_left) {
                manualTargetDeg -= DPAD_STEP_DEG;
                dpadTimer.reset();
            }
        }
    }

    private double getLoopDtSeconds() {
        long nowNs = System.nanoTime();
        if (lastLoopNs == 0L) {
            lastLoopNs = nowNs;
            return 0.02;
        }
        double dtSeconds = (nowNs - lastLoopNs) / 1_000_000_000.0;
        lastLoopNs = nowNs;
        return Math.max(1e-3, Math.min(0.1, dtSeconds));
    }

    private double clamp(double value, double min, double max) {
        if (min > max) {
            double midpoint = 0.5 * (min + max);
            min = midpoint;
            max = midpoint;
        }
        return Math.max(min, Math.min(max, value));
    }

    private double getJumpTargetDegrees(boolean highTarget) {
        double minTarget = shooter.getTurretMinTargetDegrees() + JUMP_TEST_MARGIN_DEG;
        double maxTarget = shooter.getTurretMaxTargetDegrees() - JUMP_TEST_MARGIN_DEG;
        if (minTarget > maxTarget) {
            double midpoint = 0.5 * (minTarget + maxTarget);
            minTarget = midpoint;
            maxTarget = midpoint;
        }
        return highTarget ? maxTarget : minTarget;
    }
}
