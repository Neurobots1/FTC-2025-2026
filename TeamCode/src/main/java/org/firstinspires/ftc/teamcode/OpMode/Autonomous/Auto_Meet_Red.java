package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystem.Auto_pathBuild_Blue;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.ActionScheduler;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.IndexerMode;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_GPP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_NoSort;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PGP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PPG;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Auto_Meet_Red", group = "Examples")
public class Auto_Meet_Red extends OpMode {

    private Follower follower;
    private Auto_pathBuild_Blue autoPathBuild;
    private RevColorSensorV3 colorSensor;
    private AprilTagPipeline aprilTag;

    private Indexer_Base indexer;
    private Indexer_PGP indexer_pgp;
    private Indexer_GPP indexer_gpp;
    private Indexer_PPG indexer_ppg;
    private Indexer_NoSort indexer_noSort;
    private IndexerMode activeIndexer;

    private IntakeMotor intkM;
    private LauncherSubsystem shooter;

    private final ActionScheduler scheduler = new ActionScheduler();
    private Timer actionTimer;
    private boolean routineBuilt;

    private boolean useGPPMode = false;
    private boolean usePGPMode = false;
    private boolean usePPGMode = false;
    private boolean useNoSortMode = false;
    private boolean modeLocked = false;
    private int detectedAprilTagId = -1;

    private final Pose startPose = new Pose(110, 136, Math.toRadians(90));
    private final Pose SeePatern = new Pose(92, 92, Math.toRadians(90));
    private final Pose Shoot = new Pose(92, 92, Math.toRadians(50));
    private final Pose IntkStart1 = new Pose(100, 85, Math.toRadians(0));
    private final Pose IntkFinal1 = new Pose(127, 85, Math.toRadians(0));
    private final Pose IntkStart2 = new Pose(100, 63, Math.toRadians(0));
    private final Pose IntkFinal2 = new Pose(127, 63, Math.toRadians(0));
    private final Pose ControlIntk2 = new Pose(100, 65);
    private final Pose IntkStart3 = new Pose(100, 40, Math.toRadians(0));
    private final Pose IntkFinal3 = new Pose(130, 40, Math.toRadians(0));
    private final Pose FinalShootPose = new Pose(93, 108, Math.toRadians(37));

    public static double DEFAULT_INTAKE_FINAL_SPEED_L1 = 1;
    public static double DEFAULT_INTAKE_FINAL_SPEED_L2 = 1;
    public static double DEFAULT_INTAKE_FINAL_SPEED_L3 = 1;
    public static double PGP_INTAKE_FINAL_SPEED_L1 = 1;
    public static double PGP_INTAKE_FINAL_SPEED_L2 = 1;
    public static double PGP_INTAKE_FINAL_SPEED_L3 = 1;
    public static double PPG_INTAKE_FINAL_SPEED_L1 = 1;
    public static double PPG_INTAKE_FINAL_SPEED_L2 = 1;
    public static double PPG_INTAKE_FINAL_SPEED_L3 = 1;
    public static double GPP_INTAKE_FINAL_SPEED_L1 = 1;
    public static double GPP_INTAKE_FINAL_SPEED_L2 = 1;
    public static double GPP_INTAKE_FINAL_SPEED_L3 = 1;
    public static double NOSORT_INTAKE_FINAL_SPEED_L1 = 1;
    public static double NOSORT_INTAKE_FINAL_SPEED_L2 = 1;
    public static double NOSORT_INTAKE_FINAL_SPEED_L3 = 1;

    public PathChain TakePatern, Shoot2, Shoot3, Shoot4, IntkSt1, IntkSt2, IntkSt3, IntkFi1, IntkFi2, IntkFi3;

    public void buildPaths() {
        TakePatern = follower.pathBuilder()
                .addPath(new BezierLine(startPose, SeePatern))
                .setLinearHeadingInterpolation(startPose.getHeading(), SeePatern.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal1, Shoot))
                .setLinearHeadingInterpolation(IntkFinal1.getHeading(), Shoot.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(IntkFinal2, ControlIntk2, Shoot))
                .setLinearHeadingInterpolation(IntkFinal2.getHeading(), Shoot.getHeading())
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal3, FinalShootPose))
                .setLinearHeadingInterpolation(IntkFinal3.getHeading(), FinalShootPose.getHeading())
                .build();

        IntkSt1 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart1))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart1.getHeading())
                .build();

        IntkFi1 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart1, IntkFinal1))
                .setLinearHeadingInterpolation(IntkStart1.getHeading(), IntkFinal1.getHeading())
                .build();

        IntkSt2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart2))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart2.getHeading())
                .build();

        IntkFi2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart2, IntkFinal2))
                .setLinearHeadingInterpolation(IntkStart2.getHeading(), IntkFinal2.getHeading())
                .build();

        IntkSt3 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart3))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart3.getHeading())
                .build();

        IntkFi3 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart3, IntkFinal3))
                .setLinearHeadingInterpolation(IntkStart3.getHeading(), IntkFinal3.getHeading())
                .build();
    }

    private double intakeFinalSpeedForLine(int line) {
        if (usePGPMode) {
            if (line == 1) return PGP_INTAKE_FINAL_SPEED_L1;
            if (line == 2) return PGP_INTAKE_FINAL_SPEED_L2;
            if (line == 3) return PGP_INTAKE_FINAL_SPEED_L3;
        } else if (usePPGMode) {
            if (line == 1) return PPG_INTAKE_FINAL_SPEED_L1;
            if (line == 2) return PPG_INTAKE_FINAL_SPEED_L2;
            if (line == 3) return PPG_INTAKE_FINAL_SPEED_L3;
        } else if (useGPPMode) {
            if (line == 1) return GPP_INTAKE_FINAL_SPEED_L1;
            if (line == 2) return GPP_INTAKE_FINAL_SPEED_L2;
            if (line == 3) return GPP_INTAKE_FINAL_SPEED_L3;
        } else if (useNoSortMode) {
            if (line == 1) return NOSORT_INTAKE_FINAL_SPEED_L1;
            if (line == 2) return NOSORT_INTAKE_FINAL_SPEED_L2;
            if (line == 3) return NOSORT_INTAKE_FINAL_SPEED_L3;
        }

        if (line == 1) return DEFAULT_INTAKE_FINAL_SPEED_L1;
        if (line == 2) return DEFAULT_INTAKE_FINAL_SPEED_L2;
        return DEFAULT_INTAKE_FINAL_SPEED_L3;
    }

    private boolean activeIndexerBusy() {
        return activeIndexer != null && activeIndexer.isBusy();
    }

    private void startIntakeLine(int line) {
        if (activeIndexer != null) {
            activeIndexer.startIntake(line);
        }
    }

    private void startOuttakeLine(int line) {
        if (activeIndexer != null) {
            activeIndexer.startOuttake(line);
        }
    }

    private void setActivePreSpin(boolean enabled) {
        if (activeIndexer != null) {
            activeIndexer.setPreSpinEnabled(enabled);
        }
    }

    private void lockModeTo(IndexerMode mode, String name, int aprilId) {
        activeIndexer = mode;
        modeLocked = true;
        detectedAprilTagId = aprilId;
        useGPPMode = "GPP".equals(name);
        usePGPMode = "PGP".equals(name);
        usePPGMode = "PPG".equals(name);
        useNoSortMode = "NoSort".equals(name);
    }

    private boolean attemptModeLock() {
        if (modeLocked) {
            return true;
        }

        int id = getAprilTagID();
        if (id == 21) {
            lockModeTo(indexer_gpp, "GPP", id);
            return true;
        }
        if (id == 22) {
            lockModeTo(indexer_pgp, "PGP", id);
            return true;
        }
        if (id == 23) {
            lockModeTo(indexer_ppg, "PPG", id);
            return true;
        }
        if (actionTimer.getElapsedTimeSeconds() >= 2.0) {
            lockModeTo(indexer_noSort, "NoSort", -1);
            return true;
        }
        return false;
    }

    private void buildRoutineIfNeeded() {
        if (routineBuilt) {
            return;
        }

        scheduler.setAction(Actions.sequence(
                Actions.followPath(follower, TakePatern, 1.0, true),
                Actions.instant(() -> actionTimer.resetTimer()),
                Actions.waitUntil(this::attemptModeLock),

                createIntakeTravelAction(1, IntkSt1, IntkFi1, intakeFinalSpeedForLine(1)),
                createShootCycleAction(1, Shoot2),

                createIntakeTravelAction(2, IntkSt2, IntkFi2, intakeFinalSpeedForLine(2)),
                createShootCycleAction(2, Shoot3),

                createIntakeTravelAction(3, IntkSt3, IntkFi3, intakeFinalSpeedForLine(3)),
                createShootCycleAction(3, Shoot4),

                Actions.instant(() -> setActivePreSpin(false))
        ));
        routineBuilt = true;
    }

    private org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction createIntakeTravelAction(int line,
                                                                                                     PathChain startPath,
                                                                                                     PathChain finishPath,
                                                                                                     double finishSpeed) {
        return Actions.sequence(
                Actions.instant(() -> {
                    setActivePreSpin(true);
                    startIntakeLine(line);
                }),
                Actions.followPath(follower, startPath, 1.0, true),
                Actions.followPath(follower, finishPath, finishSpeed, true),
                Actions.waitUntil(() -> !activeIndexerBusy())
        );
    }

    private org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction createShootCycleAction(int line, PathChain shotPath) {
        return Actions.sequence(
                Actions.instant(() -> setActivePreSpin(true)),
                Actions.followPath(follower, shotPath, 1.0, true),
                Actions.instant(() -> startOuttakeLine(line)),
                Actions.waitUntil(() -> !activeIndexerBusy())
        );
    }

    private int getAprilTagID() {
        if (aprilTag == null) return -1;

        List<AprilTagDetection> detections;
        try {
            detections = aprilTag.getAllDetections();
        } catch (Exception e) {
            return -1;
        }

        if (detections == null || detections.isEmpty()) return -1;

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

    @Override
    public void init() {
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        autoPathBuild = new Auto_pathBuild_Blue(follower);
        follower.setStartingPose(startPose);
        buildPaths();

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        aprilTag = new AprilTagPipeline(hardwareMap);
        aprilTag.startCamera();

        indexer = new Indexer_Base(hardwareMap);
        intkM = indexer.intkM;

        DcMotorEx flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        DcMotorEx flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter = new LauncherSubsystem(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        Servo blocker = hardwareMap.get(Servo.class, "Blocker");
        shooter.setBlocker(blocker);
        shooter.init();

        Indexer_PGP pgpCore = new Indexer_PGP(hardwareMap, indexer, shooter);
        indexer_pgp = pgpCore;
        indexer_gpp = new Indexer_GPP(pgpCore);
        indexer_ppg = new Indexer_PPG(pgpCore);
        indexer_noSort = new Indexer_NoSort(pgpCore);
        activeIndexer = null;
        routineBuilt = false;
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();
        double distance = getDistanceToGoal();

        if (activeIndexer != null) {
            activeIndexer.setShootContext(pose.getX(), pose.getY(), distance);
            activeIndexer.update();
        } else if (shooter != null) {
            shooter.updateShootingAuto(false, pose.getX(), pose.getY(), distance);
            shooter.update();
        }

        buildRoutineIfNeeded();
        scheduler.update();

        telemetry.addData("Mode", useGPPMode ? "GPP" : (usePGPMode ? "PGP" : (usePPGMode ? "PPG" : (useNoSortMode ? "NoSort" : "Normal"))));
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

    @Override
    public void stop() {
        if (intkM != null) intkM.stop();
        if (activeIndexer != null) activeIndexer.stopAll();
        if (indexer_pgp != null) indexer_pgp.stopAll();
    }

    public double getDistanceToGoal() {
        double gx = 148;
        double gy = 140;
        double dx = gx - follower.getPose().getX();
        double dy = gy - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }
}
