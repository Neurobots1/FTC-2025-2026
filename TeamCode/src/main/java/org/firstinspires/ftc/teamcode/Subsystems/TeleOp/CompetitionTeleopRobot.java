package org.firstinspires.ftc.teamcode.Subsystems.TeleOp;

import android.content.Context;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.LowPassFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.HardwareMapConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Subsystems.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ChassisHeadingLockController;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShotFeedCadenceController;
import org.firstinspires.ftc.teamcode.Constants.PedroConstants;
import org.firstinspires.ftc.teamcode.Constants.TeleopConstants;

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
    private LowPassFilter limelightXFilter;
    private LowPassFilter limelightYFilter;
    private LowPassFilter limelightHeadingFilter;
    private boolean limelightFilterInitialized;

    private final ElapsedTime aimTrimTimer = new ElapsedTime();
    private final ElapsedTime rpmTrimTimer = new ElapsedTime();
    private final ElapsedTime limelightFilterTimer = new ElapsedTime();

    private boolean fireToggleLast;
    private boolean turretRehomeButtonLast;
    private boolean tagResetButtonLast;
    private boolean turretHomeRestoredFromAuto;
    private double yawTrimDegrees;
    private double rpmTrim;
    private double goalXInches;
    private double goalYInches;
    private double aimGoalXInches;
    private double aimGoalYInches;
    private final ChassisHeadingLockController headingLockController = new ChassisHeadingLockController();
    private final ShotFeedCadenceController shotFeedController = new ShotFeedCadenceController();

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        applyAllianceGoal();
    }

    public void init(HardwareMap hardwareMap) {
        appContext = hardwareMap.appContext;
        follower = PedroConstants.createFollower(hardwareMap);
        follower.update();

        intake = new IntakeMotor(hardwareMap);
        indexerBase = new Indexer_Base(hardwareMap);
        indexerBase.StartIndexPose();

        shooter = PrecisionShooterSubsystem.create(
                hardwareMap,
                follower,
                new ShooterConstants()
        );

        applyAllianceGoal();
        initLimelight(hardwareMap);

        fireToggleLast = false;
        turretRehomeButtonLast = false;
        tagResetButtonLast = false;
        yawTrimDegrees = 0.0;
        rpmTrim = 0.0;
        headingLockController.reset();
        shotFeedController.setArmed(false);
        initLimelightFilters();
        aimTrimTimer.reset();
        rpmTrimTimer.reset();
    }

    public void startTeleop() {
        follower.startTeleopDrive();
        Pose startingPose = AutoPoseHandoff.consumePose(appContext);
        if (startingPose == null) {
            startingPose = getAutoEndPose();
        }
        follower.setStartingPose(startingPose);
        shooter.start();
        turretHomeRestoredFromAuto = shooter.restoreTurretHome(appContext);
        shotFeedController.setArmed(false);
        shooter.setGoalPosition(goalXInches, goalYInches);
        shooter.setAimPosition(aimGoalXInches, aimGoalYInches);
        limelightFilterInitialized = false;
        limelightFilterTimer.reset();
        headingLockController.reset();
        startLimelight();
    }

    public void teleopLoop(Gamepad driverGamepad,
                           Gamepad supportGamepad,
                           ElapsedTime tagResetTimer,
                           double tagCooldown,
                           JoinedTelemetry joinedTelemetry,
                           TelemetryManager telemetryManager) {
        LimelightState limelightState = updateLimelight(driverGamepad, supportGamepad, tagResetTimer, tagCooldown);
        updateDriverAimTrim(driverGamepad);
        updateTrimAdjustments(supportGamepad);
        handleTrimReset(supportGamepad);
        handleFireToggle(driverGamepad);
        handleTurretRehome(supportGamepad);

        double driveForward = -driverGamepad.left_stick_y;
        double driveStrafe = -driverGamepad.left_stick_x;
        double driverTurnCommand = -driverGamepad.right_stick_x;
        double fieldCentricOffset = AllianceSelector.Field.fieldCentricOffset(
                alliance == Alliance.BLUE
                        ? AllianceSelector.Alliance.BLUE
                        : AllianceSelector.Alliance.RED
        );

        shooter.setGoalPosition(goalXInches, goalYInches);
        shooter.setAimPosition(aimGoalXInches, aimGoalYInches);
        shooter.setYawTrimDegrees(yawTrimDegrees);
        shooter.setRpmTrim(rpmTrim);
        shooter.setSpinEnabled(TeleopConstants.ALWAYS_SPIN_FLYWHEEL || shotFeedController.isArmed());
        shooter.setAutoAimEnabled(true);
        shooter.requestFire(shotFeedController.isArmed());
        shooter.update();
        PrecisionShooterSubsystem.TelemetrySnapshot shooterSnapshot = shooter.snapshot();

        double turretDeadZoneNudge = shooter.getTurretDeadZoneNudgeTurnCommand();
        boolean nudgeDriverOverride = Math.abs(driverTurnCommand)
                >= ShooterConstants.turretDeadZoneNudgeDriverOverrideThreshold;
        double turnCommand = driverTurnCommand;
        if (!nudgeDriverOverride && turretDeadZoneNudge != 0.0) {
            turnCommand = turretDeadZoneNudge;
        }

        boolean chassisHeadingLockActive = false;
        if (TeleopConstants.ENABLE_HEADING_LOCK
                && shotFeedController.isArmed()
                && shooter.shouldUseChassisHeadingLock()) {
            turnCommand = getHeadingLockTurnCommand(shooterSnapshot);
            chassisHeadingLockActive = true;
        } else {
            headingLockController.reset();
        }

        follower.setTeleOpDrive(
                driveForward,
                driveStrafe,
                turnCommand,
                false,
                fieldCentricOffset
        );
        follower.update();

        shotFeedController.update(shooter.isFeedGateOpen(), shooterSnapshot);
        boolean forceFeedIntake = shotFeedController.shouldForceFeedIntake();

        if (!indexerBase.isBusy()) {
            if (forceFeedIntake) {
                intake.slowIntake();
            } else if (driverGamepad.right_bumper) {
                intake.intake();
            } else if (driverGamepad.left_bumper) {
                intake.outtake();
            } else {
                intake.stop();
            }
        }

        if (supportGamepad.x) {
            indexerBase.startOutTake();
        }

        if (driverGamepad.options || supportGamepad.options) {
            resetPose(getOptionsResetPose());
        }

        if (driverGamepad.share || supportGamepad.share) {
            resetPose(TeleopConstants.DRIVER_START_POSE);
        }

        indexerBase.OutTake();
        Pose pose = follower.getPose();
        joinedTelemetry.addData("Yaw Trim Deg", "%.2f", yawTrimDegrees);
        joinedTelemetry.addData("RPM Trim", "%.1f", rpmTrim);
        joinedTelemetry.addData("Turret Deadzone Target", shooter.isTurretRequestedAngleInForwardDeadZone());
        joinedTelemetry.addData("Turret Nudge Active", !nudgeDriverOverride && turretDeadZoneNudge != 0.0);
        joinedTelemetry.addData("Turret Nudge Cmd", "%.2f", turretDeadZoneNudge);
        joinedTelemetry.addData("Pose", "(%.2f, %.2f, %.1f)", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        joinedTelemetry.update();
        telemetryManager.update();
    }

    public void stop() {
        shotFeedController.setArmed(false);
        intake.stop();
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception ignored) {
            }
        }
    }

    private void handleFireToggle(Gamepad gamepad) {
        if (gamepad.y && !fireToggleLast) {
            boolean armingShooter = !shotFeedController.isArmed();
            if (armingShooter) {
                armShootingSequence();
            } else {
                stopShootingSequence();
            }
        }
        fireToggleLast = gamepad.y;
    }

    private void handleTurretRehome(Gamepad gamepad) {
        boolean rehomePressed = gamepad.a;
        if (rehomePressed && !turretRehomeButtonLast) {
            stopShootingSequence();
            shooter.requestTurretRehome();
            turretHomeRestoredFromAuto = false;
        }
        turretRehomeButtonLast = rehomePressed;
    }

    private void armShootingSequence() {
        shotFeedController.setArmed(true);
        headingLockController.reset();
    }

    private void stopShootingSequence() {
        shotFeedController.setArmed(false);
        shooter.requestFire(false);
        intake.stop();
        headingLockController.reset();
    }

    private void updateTrimAdjustments(Gamepad gamepad) {
        if (gamepad.dpad_left && aimTrimTimer.seconds() >= ShooterConstants.trimAdjustCooldownSeconds) {
            yawTrimDegrees -= ShooterConstants.aimTrimStepDeg;
            aimTrimTimer.reset();
        }

        if (gamepad.dpad_right && aimTrimTimer.seconds() >= ShooterConstants.trimAdjustCooldownSeconds) {
            yawTrimDegrees += ShooterConstants.aimTrimStepDeg;
            aimTrimTimer.reset();
        }

        if (gamepad.dpad_up && rpmTrimTimer.seconds() >= ShooterConstants.trimAdjustCooldownSeconds) {
            rpmTrim += ShooterConstants.rpmTrimStep;
            rpmTrimTimer.reset();
        }

        if (gamepad.dpad_down && rpmTrimTimer.seconds() >= ShooterConstants.trimAdjustCooldownSeconds) {
            rpmTrim -= ShooterConstants.rpmTrimStep;
            rpmTrimTimer.reset();
        }
    }

    private void handleTrimReset(Gamepad gamepad) {
        if (gamepad.y) {
            yawTrimDegrees = 0.0;
            rpmTrim = 0.0;
        }
    }

    private void updateDriverAimTrim(Gamepad gamepad) {
        if (gamepad.dpad_left && aimTrimTimer.seconds() >= ShooterConstants.trimAdjustCooldownSeconds) {
            yawTrimDegrees -= ShooterConstants.aimTrimStepDeg;
            aimTrimTimer.reset();
        }

        if (gamepad.dpad_right && aimTrimTimer.seconds() >= ShooterConstants.trimAdjustCooldownSeconds) {
            yawTrimDegrees += ShooterConstants.aimTrimStepDeg;
            aimTrimTimer.reset();
        }
    }

    private LimelightState updateLimelight(Gamepad driverGamepad,
                                           Gamepad supportGamepad,
                                           ElapsedTime tagResetTimer,
                                           double tagCooldown) {
        if (limelight == null) {
            return LimelightState.invalid(null);
        }

        LLResult result = null;
        boolean valid = false;
        boolean tagResetPressed = driverGamepad.b || supportGamepad.b;
        boolean tagResetRisingEdge = tagResetPressed && !tagResetButtonLast;
        tagResetButtonLast = tagResetPressed;

        if (tagResetRisingEdge && tagResetTimer.seconds() >= tagCooldown) {
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
        }

        if (tagResetRisingEdge
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
        headingLockController.reset();
    }

    private void initLimelight(HardwareMap hardwareMap) {
        if (!CameraConstants.LIMELIGHT_ENABLED) {
            limelight = null;
            lastLimelightPose = null;
            return;
        }

        try {
            limelight = hardwareMap.get(Limelight3A.class, HardwareMapConstants.LIMELIGHT);
            limelight.pipelineSwitch(CameraConstants.TELEOP_LIMELIGHT_PIPELINE);
            limelight.start();
        } catch (Exception ignored) {
            limelight = null;
        }
    }

    private void startLimelight() {
        if (!CameraConstants.LIMELIGHT_ENABLED || limelight == null) {
            return;
        }

        try {
            limelight.pipelineSwitch(CameraConstants.TELEOP_LIMELIGHT_PIPELINE);
            limelight.start();
        } catch (Exception ignored) {
        }
    }

    private void applyAllianceGoal() {
        goalXInches = alliance == Alliance.BLUE
                ? ShooterConstants.blueGoalXInches
                : ShooterConstants.redGoalXInches;
        goalYInches = alliance == Alliance.BLUE
                ? ShooterConstants.blueGoalYInches
                : ShooterConstants.redGoalYInches;
        aimGoalXInches = alliance == Alliance.BLUE
                ? ShooterConstants.blueHeadingAimXInches
                : ShooterConstants.redHeadingAimXInches;
        aimGoalYInches = alliance == Alliance.BLUE
                ? ShooterConstants.blueHeadingAimYInches
                : ShooterConstants.redHeadingAimYInches;

        if (shooter != null) {
            shooter.setAlliance(
                    alliance == Alliance.BLUE
                            ? PrecisionShooterSubsystem.Alliance.BLUE
                            : PrecisionShooterSubsystem.Alliance.RED
            );
            shooter.setGoalPosition(goalXInches, goalYInches);
            shooter.setAimPosition(aimGoalXInches, aimGoalYInches);
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

        Pose2D ftcPose = new Pose2D(
                DistanceUnit.METER,
                robotPose.getPosition().x,
                robotPose.getPosition().y,
                AngleUnit.RADIANS,
                robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
        );

        Pose rawPedroPose = PoseConverter.pose2DToPose(ftcPose, InvertedFTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        return filterLimelightPose(rawPedroPose);
    }

    private void initLimelightFilters() {
        double gain = CameraConstants.LIMELIGHT_POSE_FILTER_GAIN;
        limelightXFilter = new LowPassFilter(gain);
        limelightYFilter = new LowPassFilter(gain);
        limelightHeadingFilter = new LowPassFilter(gain);
        limelightFilterInitialized = false;
        limelightFilterTimer.reset();
    }

    private Pose filterLimelightPose(Pose rawPose) {
        double dtSeconds = Math.max(1e-3, Math.min(0.25, limelightFilterTimer.seconds()));
        limelightFilterTimer.reset();

        if (!limelightFilterInitialized) {
            limelightXFilter.reset(rawPose.getX(), 0.0, 0.0);
            limelightYFilter.reset(rawPose.getY(), 0.0, 0.0);
            limelightHeadingFilter.reset(rawPose.getHeading(), 0.0, 0.0);
            limelightFilterInitialized = true;
            return rawPose;
        }

        limelightXFilter.update(rawPose.getX(), dtSeconds);
        limelightYFilter.update(rawPose.getY(), dtSeconds);

        double currentHeading = limelightHeadingFilter.getState();
        double unwrappedHeading = currentHeading + normalizeRadians(rawPose.getHeading() - currentHeading);
        limelightHeadingFilter.update(unwrappedHeading, dtSeconds);

        return new Pose(
                limelightXFilter.getState(),
                limelightYFilter.getState(),
                normalizeRadians(limelightHeadingFilter.getState())
        );
    }

    private double normalizeRadians(double angle) {
        double wrapped = angle % (2.0 * Math.PI);
        if (wrapped <= -Math.PI) {
            wrapped += 2.0 * Math.PI;
        } else if (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        }
        return wrapped;
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

    private double getHeadingLockTurnCommand(PrecisionShooterSubsystem.TelemetrySnapshot shooterSnapshot) {
        return headingLockController.update(shooter, shooterSnapshot);
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
