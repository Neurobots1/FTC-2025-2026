package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.PrecisionShooterConfig;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.ActionScheduler;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.SubSystem.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.IndexerMode;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PGP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.PatternMappedIndexer;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.SortPattern;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

abstract class BaseMeetAuto extends OpMode {

    protected static final class CyclePaths {
        final PathChain startPath;
        final PathChain finishPath;
        final PathChain shotPath;
        double finishSpeed;

        CyclePaths(PathChain startPath, PathChain finishPath, PathChain shotPath, double finishSpeed) {
            this.startPath = startPath;
            this.finishPath = finishPath;
            this.shotPath = shotPath;
            this.finishSpeed = finishSpeed;
        }
    }

    protected Follower follower;
    protected RevColorSensorV3 colorSensor;
    protected AprilTagPipeline aprilTag;

    protected Indexer_Base indexerBase;
    protected Indexer_PGP indexerPgp;
    protected PatternMappedIndexer indexerGpp;
    protected PatternMappedIndexer indexerPpg;
    protected PatternMappedIndexer indexerNoSort;
    protected IndexerMode activeIndexer;

    protected IntakeMotor intake;
    protected PrecisionShooterSubsystem shooter;
    protected PrecisionShooterConfig shooterConfig;

    protected final ActionScheduler scheduler = new ActionScheduler();
    protected Timer actionTimer;

    protected boolean routineBuilt;
    protected boolean useGppMode;
    protected boolean usePgpMode;
    protected boolean usePpgMode;
    protected boolean useNoSortMode;
    protected boolean modeLocked;
    protected int detectedAprilTagId = -1;

    @Override
    public void init() {
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartPose());
        shooterConfig = new PrecisionShooterConfig();

        buildPaths();

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        aprilTag = new AprilTagPipeline(hardwareMap);
        aprilTag.startCamera();

        indexerBase = new Indexer_Base(hardwareMap);
        intake = indexerBase.intkM;

        shooter = PrecisionShooterSubsystem.create(hardwareMap, follower, shooterConfig);
        shooter.setGoalPosition(getGoalX(), getGoalY());
        shooter.setAutoAimEnabled(true);
        shooter.setSpinEnabled(false);
        shooter.requestFire(false);

        indexerPgp = new Indexer_PGP(hardwareMap, indexerBase, shooter);
        indexerGpp = new PatternMappedIndexer(indexerPgp, SortPattern.GPP);
        indexerPpg = new PatternMappedIndexer(indexerPgp, SortPattern.PPG);
        indexerNoSort = new PatternMappedIndexer(indexerPgp, SortPattern.NOSORT);
        activeIndexer = null;
        routineBuilt = false;
        modeLocked = false;
        detectedAprilTagId = -1;
        useGppMode = false;
        usePgpMode = false;
        usePpgMode = false;
        useNoSortMode = false;
    }

    @Override
    public void init_loop() {
        follower.update();
        updateShooterContext();
    }

    @Override
    public void loop() {
        follower.update();
        AutoPoseHandoff.savePose(hardwareMap.appContext, follower.getPose());
        updateShooterContext();
        buildRoutineIfNeeded();
        scheduler.update();
        updateTelemetry();
    }

    @Override
    public void stop() {
        if (follower != null) {
            AutoPoseHandoff.savePose(hardwareMap.appContext, follower.getPose());
        }
        if (intake != null) {
            intake.stop();
        }
        if (activeIndexer != null) {
            activeIndexer.stopAll();
        }
        if (indexerPgp != null) {
            indexerPgp.stopAll();
        }
    }

    protected abstract Pose getStartPose();

    protected abstract double getGoalX();

    protected abstract double getGoalY();

    protected abstract double getModeSelectTimeoutSeconds();

    protected abstract void buildPaths();

    protected abstract org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction createOpeningAction();

    protected abstract CyclePaths[] getCyclePaths();

    protected abstract double getIntakeFinishSpeed(int line);

    protected abstract String getAutoLabel();

    protected final boolean autoUsesChassisAim() {
        return shooterConfig != null
                && !shooterConfig.turretEnabled
                && shooterConfig.lockChassisHeadingWhenTurretDisabled;
    }

    protected final PathBuilder applyAutoHeading(PathBuilder builder, Pose startPose, Pose endPose, boolean faceGoalWhenTurretDisabled) {
        if (faceGoalWhenTurretDisabled && autoUsesChassisAim()) {
            return builder.setHeadingInterpolation(HeadingInterpolator.facingPoint(getGoalX(), getGoalY()));
        }
        return builder.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
    }

    protected final void buildRoutineIfNeeded() {
        if (routineBuilt) {
            return;
        }

        CyclePaths[] cycles = getCyclePaths();
        org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction[] cycleActions =
                new org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction[cycles.length];
        for (int i = 0; i < cycles.length; i++) {
            cycles[i].finishSpeed = getIntakeFinishSpeed(i + 1);
            cycleActions[i] = createCycleAction(i + 1, cycles[i]);
        }

        org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction openingAction = createOpeningAction();
        org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction fullRoutine = Actions.sequence(
                openingAction,
                Actions.instant(() -> actionTimer.resetTimer()),
                Actions.waitUntil(this::attemptModeLock),
                Actions.sequence(cycleActions),
                Actions.instant(() -> setActivePreSpin(false))
        );

        scheduler.setAction(fullRoutine);
        routineBuilt = true;
    }

    protected final org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction createIntakeTravelAction(int line, CyclePaths cycle) {
        return Actions.sequence(
                Actions.instant(() -> {
                    setActivePreSpin(true);
                    startIntakeLine(line);
                }),
                Actions.followPath(follower, cycle.startPath, 1.0, true),
                Actions.followPath(follower, cycle.finishPath, cycle.finishSpeed, true),
                Actions.waitUntil(() -> !activeIndexerBusy())
        );
    }

    protected final org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction createShootCycleAction(int line, PathChain shotPath) {
        return Actions.sequence(
                Actions.instant(() -> setActivePreSpin(true)),
                Actions.followPath(follower, shotPath, 1.0, true),
                Actions.instant(() -> startOuttakeLine(line)),
                Actions.waitUntil(() -> !activeIndexerBusy())
        );
    }

    private org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction createCycleAction(int line, CyclePaths cycle) {
        return Actions.sequence(
                createIntakeTravelAction(line, cycle),
                createShootCycleAction(line, cycle.shotPath)
        );
    }

    protected final boolean activeIndexerBusy() {
        return activeIndexer != null && activeIndexer.isBusy();
    }

    protected final void startIntakeLine(int line) {
        if (activeIndexer != null) {
            activeIndexer.startIntake(line);
        }
    }

    protected final void startOuttakeLine(int line) {
        if (activeIndexer != null) {
            activeIndexer.startOuttake(line);
        }
    }

    protected final void setActivePreSpin(boolean enabled) {
        if (activeIndexer != null) {
            activeIndexer.setPreSpinEnabled(enabled);
        }
    }

    protected final void lockModeTo(IndexerMode mode, String name, int aprilId) {
        activeIndexer = mode;
        modeLocked = true;
        detectedAprilTagId = aprilId;
        useGppMode = "GPP".equals(name);
        usePgpMode = "PGP".equals(name);
        usePpgMode = "PPG".equals(name);
        useNoSortMode = "NoSort".equals(name);
    }

    protected final boolean attemptModeLock() {
        if (modeLocked) {
            return true;
        }

        int id = getAprilTagID();
        if (id == 21) {
            lockModeTo(indexerGpp, SortPattern.GPP.displayName(), id);
            return true;
        }
        if (id == 22) {
            lockModeTo(indexerPgp, SortPattern.PGP.displayName(), id);
            return true;
        }
        if (id == 23) {
            lockModeTo(indexerPpg, SortPattern.PPG.displayName(), id);
            return true;
        }
        if (actionTimer.getElapsedTimeSeconds() >= getModeSelectTimeoutSeconds()) {
            lockModeTo(indexerNoSort, SortPattern.NOSORT.displayName(), -1);
            return true;
        }
        return false;
    }

    protected final int getAprilTagID() {
        if (aprilTag == null) {
            return -1;
        }

        List<AprilTagDetection> detections;
        try {
            detections = aprilTag.getAllDetections();
        } catch (Exception e) {
            return -1;
        }

        if (detections == null || detections.isEmpty()) {
            return -1;
        }

        for (AprilTagDetection det : detections) {
            int id = det.id;
            if (id == 21 || id == 22 || id == 23) {
                aprilTag.stopCamera();
                aprilTag = null;
                return id;
            }
        }
        return -1;
    }

    protected final double getDistanceToGoal() {
        double dx = getGoalX() - follower.getPose().getX();
        double dy = getGoalY() - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }

    private void updateShooterContext() {
        Pose pose = follower.getPose();
        double distance = getDistanceToGoal();

        if (shooter != null) {
            shooter.setGoalPosition(getGoalX(), getGoalY());
        }

        if (activeIndexer != null) {
            activeIndexer.setShootContext(pose.getX(), pose.getY(), distance);
            activeIndexer.update();
        } else if (indexerPgp != null) {
            indexerPgp.setShootContext(pose.getX(), pose.getY(), distance);
            indexerPgp.update();
        } else if (shooter != null) {
            shooter.setSpinEnabled(false);
            shooter.requestFire(false);
            shooter.update();
        }
    }

    private void updateTelemetry() {
        Pose pose = follower.getPose();
        telemetry.addData("Auto", getAutoLabel());
        telemetry.addData("Mode", useGppMode ? "GPP" : (usePgpMode ? "PGP" : (usePpgMode ? "PPG" : (useNoSortMode ? "NoSort" : "Normal"))));
        telemetry.addData("AprilTag ID", detectedAprilTagId);
        telemetry.addData("Routine Idle", scheduler.isIdle());
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading", Math.toDegrees(pose.getHeading()));
        telemetry.addData("IndexerBusy", activeIndexerBusy());
        telemetry.addData("ColorSensor Distance", colorSensor.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
}
