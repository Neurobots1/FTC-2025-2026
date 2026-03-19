package org.firstinspires.ftc.teamcode.SubSystem.TeleOp;

import android.content.Context;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants.CompetitionTeleopConstants;
import org.firstinspires.ftc.teamcode.Constants.PrecisionShooterConfig;
import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector;
import org.firstinspires.ftc.teamcode.SubSystem.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public final class CompetitionTeleopRobot {

    public enum Alliance {
        BLUE,
        RED
    }

    private Alliance alliance = Alliance.BLUE;

    private Context appContext;
    private Follower follower;
    private IntakeMotor intake;
    private Indexer_Base indexerBase;
    private PrecisionShooterSubsystem shooter;
    private Limelight3A limelight;
    private Pose lastLimelightPose;

    private final ElapsedTime goalUpTimer = new ElapsedTime();
    private final ElapsedTime goalDownTimer = new ElapsedTime();

    private boolean shooterEnabled;
    private boolean shootToggleLast;
    private double goalXInches;
    private double goalYInches;

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        applyAllianceGoal();
    }

    public void init(HardwareMap hardwareMap) {
        appContext = hardwareMap.appContext;
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        intake = new IntakeMotor(hardwareMap);
        indexerBase = new Indexer_Base(hardwareMap);
        indexerBase.StartIndexPose();

        shooter = PrecisionShooterSubsystem.create(
                hardwareMap,
                follower,
                new PrecisionShooterConfig()
        );

        applyAllianceGoal();
        initLimelight(hardwareMap);

        shooterEnabled = false;
        shootToggleLast = false;
        goalUpTimer.reset();
        goalDownTimer.reset();
    }

    public void startTeleop() {
        follower.startTeleopDrive();
        Pose startingPose = AutoPoseHandoff.consumePose(appContext);
        if (startingPose == null) {
            startingPose = getAutoEndPose();
        }
        follower.setStartingPose(startingPose);
        shooter.start();
        shooter.setGoalPosition(goalXInches, goalYInches);
        startLimelight();
    }

    public void teleopLoop(Gamepad gamepad,
                           ElapsedTime tagResetTimer,
                           double tagCooldown,
                           JoinedTelemetry joinedTelemetry,
                           TelemetryManager telemetryManager) {
        follower.update();

        LimelightState limelightState = updateLimelight(gamepad, tagResetTimer, tagCooldown);
        updateGoalAdjustments(gamepad);
        handleShooterToggle(gamepad);

        shooter.setGoalPosition(goalXInches, goalYInches);
        shooter.setSpinEnabled(shooterEnabled);
        shooter.setAutoAimEnabled(shooterEnabled);
        shooter.requestFire(shooterEnabled && gamepad.right_bumper);
        shooter.update();

        double turnCommand = -gamepad.right_stick_x;
        boolean chassisHeadingLockActive = false;
        if (shooterEnabled
                && shooter.shouldUseChassisHeadingLock()
                && Math.abs(gamepad.right_stick_x) < shooter.getChassisAimManualOverrideThreshold()) {
            turnCommand = shooter.getChassisAimTurnCommand();
            chassisHeadingLockActive = true;
        }

        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                turnCommand,
                false,
                AllianceSelector.Field.fieldCentricOffset(
                        alliance == Alliance.BLUE
                                ? AllianceSelector.Alliance.BLUE
                                : AllianceSelector.Alliance.RED
                )
        );

        if (!indexerBase.isBusy()) {
            if (gamepad.right_bumper) {
                intake.intake();
            } else if (gamepad.left_bumper) {
                intake.outtake();
            } else {
                intake.stop();
            }
        }

        if (gamepad.dpad_left) {
            indexerBase.startOutTake();
        }

        if (gamepad.options) {
            resetPose(getOptionsResetPose());
        }

        if (gamepad.share) {
            resetPose(CompetitionTeleopConstants.DRIVER_START_POSE);
        }

        indexerBase.OutTake();

        PrecisionShooterSubsystem.TelemetrySnapshot shooterSnapshot = shooter.snapshot();
        Pose pose = follower.getPose();

        joinedTelemetry.addData("Alliance", alliance);
        joinedTelemetry.addData("Shooter Enabled", shooterEnabled);
        joinedTelemetry.addData("Turret Enabled", shooter.isTurretEnabled());
        joinedTelemetry.addData("Chassis Aim Lock", chassisHeadingLockActive);
        joinedTelemetry.addData("Shot Zone", shooterSnapshot.inShootingZone);
        joinedTelemetry.addData("Ready", shooterSnapshot.ready);
        joinedTelemetry.addData("Homed", shooterSnapshot.homed);
        joinedTelemetry.addData("Target RPM", "%.1f", shooterSnapshot.targetRpm);
        joinedTelemetry.addData("Actual RPM", "%.1f", shooterSnapshot.actualRpm);
        joinedTelemetry.addData("Hood", "%.2f / %.2f", shooterSnapshot.nominalHoodDeg, shooterSnapshot.compensatedHoodDeg);
        joinedTelemetry.addData("Turret", "%.2f / %.2f", shooterSnapshot.turretAngleDeg, shooterSnapshot.turretTargetDeg);
        joinedTelemetry.addData("Aim Error Deg", "%.2f", Math.toDegrees(shooter.getChassisHeadingErrorRadians()));
        joinedTelemetry.addData("Goal", "(%.1f, %.1f)", goalXInches, goalYInches);
        joinedTelemetry.addData("X", pose.getX());
        joinedTelemetry.addData("Y", pose.getY());
        joinedTelemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
        joinedTelemetry.addData("Indexer Busy", indexerBase.isBusy());
        joinedTelemetry.addData("LL Result", limelightState.result == null ? "null" : "non-null");
        joinedTelemetry.addData("LL Valid", limelightState.valid);
        joinedTelemetry.addData("LL Pose Cached", lastLimelightPose != null);
        joinedTelemetry.addData("Shooter Status", shooterSnapshot.status);
        joinedTelemetry.update();
        telemetryManager.update();
    }

    public void stop() {
        intake.stop();
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception ignored) {
            }
        }
    }

    private void handleShooterToggle(Gamepad gamepad) {
        if (gamepad.y && !shootToggleLast) {
            shooterEnabled = !shooterEnabled;
        }
        shootToggleLast = gamepad.y;
    }

    private void updateGoalAdjustments(Gamepad gamepad) {
        if (gamepad.dpad_up && goalUpTimer.seconds() >= CompetitionTeleopConstants.GOAL_ADJUST_COOLDOWN_S) {
            goalXInches += CompetitionTeleopConstants.GOAL_X_STEP_UP_IN;
            goalUpTimer.reset();
        }

        if (gamepad.dpad_down && goalDownTimer.seconds() >= CompetitionTeleopConstants.GOAL_ADJUST_COOLDOWN_S) {
            goalXInches -= CompetitionTeleopConstants.GOAL_X_STEP_DOWN_IN;
            goalDownTimer.reset();
        }
    }

    private LimelightState updateLimelight(Gamepad gamepad,
                                           ElapsedTime tagResetTimer,
                                           double tagCooldown) {
        if (limelight == null) {
            return LimelightState.invalid(null);
        }

        LLResult result = null;
        boolean valid = false;

        try {
            result = limelight.getLatestResult();
            valid = result != null && result.isValid();

            if (valid) {
                Pose limelightPose = convertLimelightResultToPedroPose(result);
                if (limelightPose != null) {
                    lastLimelightPose = limelightPose;
                }
            }
        } catch (Exception ignored) {
            valid = false;
        }

        if (gamepad.b
                && tagResetTimer.seconds() >= tagCooldown
                && lastLimelightPose != null) {
            resetPose(lastLimelightPose);
            tagResetTimer.reset();
        }

        return new LimelightState(result, valid);
    }

    private void resetPose(Pose pose) {
        follower.setPose(pose);
        shooter.notifyPoseJump();
    }

    private void initLimelight(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(CompetitionTeleopConstants.LIMELIGHT_PIPELINE);
            limelight.start();
        } catch (Exception ignored) {
            limelight = null;
        }
    }

    private void startLimelight() {
        if (limelight == null) {
            return;
        }

        try {
            limelight.pipelineSwitch(CompetitionTeleopConstants.LIMELIGHT_PIPELINE);
            limelight.start();
        } catch (Exception ignored) {
        }
    }

    private void applyAllianceGoal() {
        if (alliance == Alliance.BLUE) {
            goalXInches = CompetitionTeleopConstants.BLUE_GOAL_X_IN;
            goalYInches = CompetitionTeleopConstants.BLUE_GOAL_Y_IN;
        } else {
            goalXInches = CompetitionTeleopConstants.RED_GOAL_X_IN;
            goalYInches = CompetitionTeleopConstants.RED_GOAL_Y_IN;
        }

        if (shooter != null) {
            shooter.setAlliance(
                    alliance == Alliance.BLUE
                            ? PrecisionShooterSubsystem.Alliance.BLUE
                            : PrecisionShooterSubsystem.Alliance.RED
            );
            shooter.setGoalPosition(goalXInches, goalYInches);
        }
    }

    private Pose convertLimelightResultToPedroPose(LLResult result) {
        if (result == null || !result.isValid()) {
            return null;
        }

        Pose3D robotPose = result.getBotpose();
        if (robotPose == null) {
            return null;
        }

        double yawDeg = robotPose.getOrientation().getYaw(AngleUnit.DEGREES) + 270.0;
        yawDeg = (yawDeg % 360.0 + 360.0) % 360.0;

        double xIn = (robotPose.getPosition().y * CompetitionTeleopConstants.INCHES_PER_METER)
                + CompetitionTeleopConstants.FIELD_OFFSET_IN;
        double yIn = (-robotPose.getPosition().x * CompetitionTeleopConstants.INCHES_PER_METER)
                + CompetitionTeleopConstants.FIELD_OFFSET_IN;

        return new Pose(xIn, yIn, Math.toRadians(yawDeg));
    }

    private Pose getOptionsResetPose() {
        AllianceSelector.Alliance selectedAlliance =
                alliance == Alliance.BLUE
                        ? AllianceSelector.Alliance.BLUE
                        : AllianceSelector.Alliance.RED;

        return new Pose(
                AllianceSelector.Field.resetX(selectedAlliance),
                AllianceSelector.Field.resetY(selectedAlliance),
                Math.toRadians(270)
        );
    }

    private Pose getAutoEndPose() {
        AllianceSelector.Alliance selectedAlliance =
                alliance == Alliance.BLUE
                        ? AllianceSelector.Alliance.BLUE
                        : AllianceSelector.Alliance.RED;

        return new Pose(
                AllianceSelector.Field.EndAutoX(selectedAlliance),
                AllianceSelector.Field.EndAutoY(selectedAlliance),
                Math.toRadians(AllianceSelector.Field.EndAutoH(selectedAlliance))
        );
    }

    private static final class LimelightState {
        final LLResult result;
        final boolean valid;

        LimelightState(LLResult result, boolean valid) {
            this.result = result;
            this.valid = valid;
        }

        static LimelightState invalid(LLResult result) {
            return new LimelightState(result, false);
        }
    }
}
