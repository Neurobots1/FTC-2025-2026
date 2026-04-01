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
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AllianceMirroring;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.Constants.PedroConstants;
import org.firstinspires.ftc.teamcode.Constants.TeleopConstants;

public final class CompetitionTeleopRobot {

    public enum Alliance {
        BLUE,
        RED
    }

    private enum ShootFeedState {
        IDLE,
        ARMED_WAITING_READY,
        GATE_OPENING,
        FEEDING
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
    private final ElapsedTime blockerOpenTimer = new ElapsedTime();
    private final ElapsedTime limelightFilterTimer = new ElapsedTime();

    private boolean feedingEnabled;
    private boolean fireToggleLast;
    private boolean blockerOpenCommanded;
    private double yawTrimDegrees;
    private double rpmTrim;
    private double goalXInches;
    private double goalYInches;
    private double aimGoalXInches;
    private double aimGoalYInches;
    private ShootFeedState shootFeedState = ShootFeedState.IDLE;

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

        feedingEnabled = false;
        fireToggleLast = false;
        blockerOpenCommanded = false;
        shootFeedState = ShootFeedState.IDLE;
        yawTrimDegrees = 0.0;
        rpmTrim = 0.0;
        initLimelightFilters();
        aimTrimTimer.reset();
        rpmTrimTimer.reset();
        blockerOpenTimer.reset();
    }

    public void startTeleop() {
        follower.startTeleopDrive();
        Pose startingPose = AutoPoseHandoff.consumePose(appContext);
        if (startingPose == null) {
            startingPose = getAutoEndPose();
        }
        follower.setStartingPose(startingPose);
        shooter.start();
        setShootFeedState(ShootFeedState.IDLE);
        shooter.setGoalPosition(goalXInches, goalYInches);
        shooter.setAimPosition(aimGoalXInches, aimGoalYInches);
        limelightFilterInitialized = false;
        limelightFilterTimer.reset();
        startLimelight();
    }

    public void teleopLoop(Gamepad driverGamepad,
                           Gamepad supportGamepad,
                           ElapsedTime tagResetTimer,
                           double tagCooldown,
                           JoinedTelemetry joinedTelemetry,
                           TelemetryManager telemetryManager) {
        follower.update();

        LimelightState limelightState = updateLimelight(driverGamepad, supportGamepad, tagResetTimer, tagCooldown);
        updateTrimAdjustments(supportGamepad);
        handleTrimReset(supportGamepad);
        handleFireToggle(driverGamepad);

        shooter.setGoalPosition(goalXInches, goalYInches);
        shooter.setAimPosition(aimGoalXInches, aimGoalYInches);
        shooter.setYawTrimDegrees(yawTrimDegrees);
        shooter.setRpmTrim(rpmTrim);
        shooter.setSpinEnabled(TeleopConstants.ALWAYS_SPIN_FLYWHEEL || feedingEnabled);
        shooter.setAutoAimEnabled(true);
        shooter.requestFire(feedingEnabled);
        shooter.update();

        double turnCommand = -driverGamepad.right_stick_x;
        boolean chassisHeadingLockActive = false;
        if (TeleopConstants.ENABLE_HEADING_LOCK
                && feedingEnabled
                && shooter.shouldUseChassisHeadingLock()) {
            turnCommand = shooter.getChassisAimTurnCommand() * TeleopConstants.HEADING_LOCK_TURN_SCALE;
            chassisHeadingLockActive = true;
        }

        follower.setTeleOpDrive(
                -driverGamepad.left_stick_y,
                -driverGamepad.left_stick_x,
                turnCommand,
                false,
                AllianceSelector.Field.fieldCentricOffset(
                        alliance == Alliance.BLUE
                                ? AllianceSelector.Alliance.BLUE
                                : AllianceSelector.Alliance.RED
                )
        );

        updateShootFeedState(shooter.isFeedGateOpen());
        boolean blockerSettledOpen = shootFeedState == ShootFeedState.FEEDING;
        boolean forceFeedIntake = blockerSettledOpen;

        if (!indexerBase.isBusy()) {
            if (forceFeedIntake) {
                intake.intake();
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

        PrecisionShooterSubsystem.TelemetrySnapshot shooterSnapshot = shooter.snapshot();
        Pose pose = follower.getPose();

        joinedTelemetry.addData("Alliance", alliance);
        joinedTelemetry.addData("Shoot Armed", feedingEnabled);
        joinedTelemetry.addData("Feed State", shootFeedState);
        joinedTelemetry.addData("Always Spin", TeleopConstants.ALWAYS_SPIN_FLYWHEEL);
        joinedTelemetry.addData("Heading Lock Enabled", TeleopConstants.ENABLE_HEADING_LOCK);
        joinedTelemetry.addData("Heading Lock Scale", "%.2f", TeleopConstants.HEADING_LOCK_TURN_SCALE);
        joinedTelemetry.addData("Turret Enabled", shooter.isTurretEnabled());
        joinedTelemetry.addData("Chassis Aim Lock", chassisHeadingLockActive);
        joinedTelemetry.addData("Driver 1", "Drive / Intake / Shoot");
        joinedTelemetry.addData("Driver 2", "Trim / Reset / Index Empty");
        joinedTelemetry.addData("Shot Zone", shooterSnapshot.inShootingZone);
        joinedTelemetry.addData("Ready", shooterSnapshot.ready);
        joinedTelemetry.addData("Forced Feed Intake", forceFeedIntake);
        joinedTelemetry.addData("Blocker Open", shooter.isFeedGateOpen());
        joinedTelemetry.addData("Blocker Settled Open", blockerSettledOpen);
        joinedTelemetry.addData("Homed", shooterSnapshot.homed);
        joinedTelemetry.addData("Target RPM", "%.1f", shooterSnapshot.targetRpm);
        joinedTelemetry.addData("Actual RPM", "%.1f", shooterSnapshot.actualRpm);
        joinedTelemetry.addData("Table Dist", "%.2f", shooterSnapshot.tableDistanceInches);
        joinedTelemetry.addData("Hood", "%.2f / %.2f", shooterSnapshot.nominalHoodDeg, shooterSnapshot.compensatedHoodDeg);
        joinedTelemetry.addData("Turret", "%.2f / %.2f", shooterSnapshot.turretAngleDeg, shooterSnapshot.turretTargetDeg);
        joinedTelemetry.addData("Aim Error Deg", "%.2f", Math.toDegrees(shooter.getAdjustedChassisHeadingErrorRadians()));
        joinedTelemetry.addData("Aim Turn Cmd", "%.3f", shooter.getLastChassisAimTurnCommand());
        joinedTelemetry.addData("Aim PIDF", "P %.3f I %.3f D %.3f F %.3f",
                shooter.getLastChassisAimProportionalTerm(),
                shooter.getLastChassisAimIntegralTerm(),
                shooter.getLastChassisAimDerivativeTerm(),
                shooter.getLastChassisAimFeedforwardTerm());
        joinedTelemetry.addData("Robot Omega Deg/S", "%.2f", Math.toDegrees(shooter.getRobotOmegaRadiansPerSecond()));
        joinedTelemetry.addData("Yaw Trim Deg", "%.2f", yawTrimDegrees);
        joinedTelemetry.addData("RPM Trim", "%.1f", rpmTrim);
        joinedTelemetry.addData("Goal", "(%.1f, %.1f)", goalXInches, goalYInches);
        joinedTelemetry.addData("Aim Goal", "(%.1f, %.1f)", aimGoalXInches, aimGoalYInches);
        joinedTelemetry.addData("X", pose.getX());
        joinedTelemetry.addData("Y", pose.getY());
        joinedTelemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
        joinedTelemetry.addData("Indexer Busy", indexerBase.isBusy());
        joinedTelemetry.addData("LL Enabled", CameraConstants.LIMELIGHT_ENABLED);
        joinedTelemetry.addData("LL Result", limelightState.result == null ? "null" : "non-null");
        joinedTelemetry.addData("LL Valid", limelightState.valid);
        joinedTelemetry.addData("LL Pose Cached", lastLimelightPose != null);
        joinedTelemetry.addData("Controls", "D1 Y=arm/cancel shoot, RB/LB=intake | D2 DpadL/R=aim trim, DpadU/D=RPM trim, Y=zero trim");
        joinedTelemetry.addData("Resets", "D1/D2 B=tag reset, options=alliance reset, share=driver start | D2 X=indexer empty");
        joinedTelemetry.addData("Shooter Status", shooterSnapshot.status);
        joinedTelemetry.update();
        telemetryManager.update();
    }

    public void stop() {
        setShootFeedState(ShootFeedState.IDLE);
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
            setShootFeedState(feedingEnabled
                    ? ShootFeedState.IDLE
                    : ShootFeedState.ARMED_WAITING_READY);
        }
        fireToggleLast = gamepad.y;
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

    private void updateShootFeedState(boolean gateOpen) {
        if (!feedingEnabled) {
            blockerOpenCommanded = false;
            shootFeedState = ShootFeedState.IDLE;
            return;
        }

        if (!gateOpen) {
            blockerOpenCommanded = false;
            if (shootFeedState != ShootFeedState.ARMED_WAITING_READY) {
                shootFeedState = ShootFeedState.ARMED_WAITING_READY;
            }
            return;
        }

        blockerOpenCommanded = true;
        if (shootFeedState == ShootFeedState.ARMED_WAITING_READY) {
            shootFeedState = ShootFeedState.GATE_OPENING;
            blockerOpenTimer.reset();
            return;
        }

        if (shootFeedState == ShootFeedState.GATE_OPENING
                && blockerOpenTimer.seconds() >= ShooterConstants.feedOpenSettlingSeconds) {
            shootFeedState = ShootFeedState.FEEDING;
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

        if ((driverGamepad.b || supportGamepad.b)
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
        Pose goalPose = AllianceMirroring.forAlliance(
                TeleopConstants.BLUE_GOAL_POSE,
                alliance == Alliance.BLUE ? AutoAlliance.BLUE : AutoAlliance.RED
        );
        Pose aimGoalPose = AllianceMirroring.forAlliance(
                TeleopConstants.BLUE_HEADING_AIM_POSE,
                alliance == Alliance.BLUE ? AutoAlliance.BLUE : AutoAlliance.RED
        );
        goalXInches = goalPose.getX();
        goalYInches = goalPose.getY();
        aimGoalXInches = aimGoalPose.getX();
        aimGoalYInches = aimGoalPose.getY();

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

    private void setShootFeedState(ShootFeedState nextState) {
        shootFeedState = nextState;
        feedingEnabled = nextState != ShootFeedState.IDLE;
        if (nextState != ShootFeedState.GATE_OPENING) {
            blockerOpenTimer.reset();
        }
        if (nextState == ShootFeedState.IDLE) {
            blockerOpenCommanded = false;
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
