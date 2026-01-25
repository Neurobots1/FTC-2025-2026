package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.SubSystem.Auto_pathBuild_Blue;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.IndexerMode;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_GPP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PPG;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PGP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_NoSort;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Autonomous(name = "Auto_blue1", group = "Examples")
public class Auto_Blue1 extends OpMode {

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

    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;

    private VoltageSensor voltageSensor;
    private LauncherSubsystem Shooter;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private TelemetryManager telemetryM;

    private boolean useGPPMode = false;
    private boolean usePGPMode = false;
    private boolean usePPGMode = false;
    private boolean useNoSortMode = false;

    private boolean modeLocked = false;
    private int detectedAprilTagId = -1;

    public final Pose startPose = new Pose(33, 136, Math.toRadians(88));
    private final Pose SeePatern = new Pose(47.1, 92, Math.toRadians(70));
    private final Pose Shoot = new Pose(47, 92, Math.toRadians(130));
    private final Pose IntkStart1 = new Pose(42, 83, Math.toRadians(180));
    private final Pose IntkFinal1 = new Pose(14.5, 83, Math.toRadians(180));
    private final Pose IntkStart2 = new Pose(42, 61, Math.toRadians(180));
    private final Pose IntkFinal2 = new Pose(10, 61, Math.toRadians(180));
    private final Pose IntkStart3 = new Pose(42, 40, Math.toRadians(180));
    private final Pose IntkFinal3 = new Pose(7, 37.05, Math.toRadians(180));
    private final Pose FinalShootPose = new Pose(55, 105, Math.toRadians(140));

    public PathChain TakePatern, Shoot1, Shoot2, Shoot3, Shoot4,
            IntkSt1, IntkSt2, IntkSt3, IntkFi1, IntkFi2, IntkFi3;

    public void buildPaths() {
        TakePatern = follower.pathBuilder()
                .addPath(new BezierLine(startPose, SeePatern))
                .setLinearHeadingInterpolation(startPose.getHeading(), SeePatern.getHeading())
                .build();

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(SeePatern, Shoot))
                .setLinearHeadingInterpolation(SeePatern.getHeading(), Shoot.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal1, Shoot))
                .setLinearHeadingInterpolation(IntkFinal1.getHeading(), Shoot.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal2, Shoot))
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

    private boolean activeIndexerBusy() {
        return activeIndexer != null && activeIndexer.isBusy();
    }

    private void startIntakeLine(int line) {
        if (activeIndexer != null) activeIndexer.startIntake(line);
    }

    private void startOuttakeLine(int line) {
        if (activeIndexer != null) activeIndexer.startOuttake(line);
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

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(TakePatern, 1, true);
                    setPathState(1);
                }
                break;

            case 1:
                if (!modeLocked) {
                    int id = getAprilTagID();

                    if (id == 21) {
                        lockModeTo(indexer_gpp, "GPP", id);
                        setPathState(2);
                    } else if (id == 22) {
                        lockModeTo(indexer_pgp, "PGP", id);
                        setPathState(2);
                    } else if (id == 23) {
                        lockModeTo(indexer_ppg, "PPG", id);
                        setPathState(2);
                    } else if (pathTimer.getElapsedTimeSeconds() >= 4.0) {
                        lockModeTo(indexer_noSort, "NoSort", -1);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                follower.followPath(Shoot1, 1, true);
                setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    startOuttakeLine(1);
                    setPathState(4000);
                }
                break;

            case 4000:
                if (!activeIndexerBusy()) {
                    startIntakeLine(1);
                    follower.followPath(IntkSt1, 1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi1, 0.4, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && !activeIndexerBusy()) {
                    follower.followPath(Shoot2, 1, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    startOuttakeLine(1);
                    setPathState(7);
                }
                break;

            case 7:
                if (!activeIndexerBusy()) {
                    follower.followPath(IntkSt2, 1, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    startIntakeLine(2);
                    follower.followPath(IntkFi2, 0.4, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy() && !activeIndexerBusy()) {
                    follower.followPath(Shoot3, 1, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    startOuttakeLine(2);
                    setPathState(11);
                }
                break;

            case 11:
                if (!activeIndexerBusy()) {
                    follower.followPath(IntkSt3, 1, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    startIntakeLine(3);
                    follower.followPath(IntkFi3, 0.4, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy() && !activeIndexerBusy()) {
                    follower.followPath(Shoot4, 1, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    startOuttakeLine(3);
                    setPathState(15);
                }
                break;

            case 15:
                if (!activeIndexerBusy()) {
                    setPathState(-1);
                }
                break;

            case -1:
                break;
        }
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
    public void stop() {
        if (intkM != null) intkM.stop();

        if (activeIndexer != null) activeIndexer.stopAll();

        if (aprilTag != null) {
            try { aprilTag.stopCamera(); } catch (Exception ignoredv) {}
        }
    }




    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
        if (actionTimer != null) actionTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.getPose();
        double distance = getDistanceToGoal();

        if (activeIndexer != null) activeIndexer.setShootContext(pose.getX(), pose.getY(), distance);

        if (activeIndexer != null) {
            activeIndexer.update();
        } else {
            if (Shooter != null) {
                Shooter.updateShootingAuto(false, pose.getX(), pose.getY(), distance);
                Shooter.update();
            }
        }

        autonomousPathUpdate();

        telemetry.addData("Mode", useGPPMode ? "GPP" : (usePGPMode ? "PGP" : (usePPGMode ? "PPG" : (useNoSortMode ? "NoSort" : "Normal"))));
        telemetry.addData("AprilTag ID", detectedAprilTagId);
        telemetry.addData("path state", pathState);
        telemetry.addData("followerBusy", follower.isBusy());

        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading", Math.toDegrees(pose.getHeading()));

        telemetry.addData("IndexerBusy", activeIndexerBusy());
        telemetry.addData("ColorSensor Distance", colorSensor.getDistance(DistanceUnit.MM));

        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        autoPathBuild = new Auto_pathBuild_Blue(follower);
        follower.setStartingPose(startPose);

        buildPaths();

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        aprilTag = new AprilTagPipeline(hardwareMap);
        aprilTag.startCamera();

        indexer = new Indexer_Base(hardwareMap);
        intkM = indexer.intkM;

        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Shooter = new LauncherSubsystem(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        Servo blocker = hardwareMap.get(Servo.class, "Blocker");
        Shooter.setBlocker(blocker);
        Shooter.init();

        Indexer_PGP pgpCore = new Indexer_PGP(hardwareMap, indexer, Shooter);

        indexer_pgp = pgpCore;
        indexer_gpp = new Indexer_GPP(pgpCore);
        indexer_ppg = new Indexer_PPG(pgpCore);
        indexer_noSort = new Indexer_NoSort(pgpCore);


        activeIndexer = null;
        setPathState(0);
    }

    public double getDistanceToGoal() {
        double gx = 12;
        double gy = 132;
        double dx = gx - follower.getPose().getX();
        double dy = gy - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }
}
