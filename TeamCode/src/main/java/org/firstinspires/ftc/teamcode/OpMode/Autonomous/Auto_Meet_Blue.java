package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

import org.firstinspires.ftc.teamcode.SubSystem.Auto_Pose;
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

@Autonomous
public class Auto_Meet_Blue extends OpMode {

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
    public Auto_Pose autoPose;

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
    private boolean wantShoot = false;
    private int detectedAprilTagId = -1;

    public final Pose startPose = new Pose(20, 119, Math.toRadians(139));
    private final Pose Shoot = new Pose(47, 92, Math.toRadians(139));
    private final Pose IntkStart1 = new Pose(50, 88, Math.toRadians(190));
    private final Pose IntkFinal1 = new Pose(23, 82, Math.toRadians(190));
    private final Pose IntkStart2 = new Pose(50, 55, Math.toRadians(180));
    private final Pose IntkFinal2 = new Pose(15, 55, Math.toRadians(180));
    private final Pose ControlIntk2 = new Pose(45, 55);
    private final Pose IntkStart3 = new Pose(50, 38, Math.toRadians(180));
    private final Pose IntkFinal3 = new Pose(8, 38, Math.toRadians(180));
    private final Pose FinalShootPose = new Pose(55, 105, Math.toRadians(145));
    private final Pose GateOpen = new Pose(15, 73, Math.toRadians(90));

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

    public PathChain OpenGate, TakePatern, ShootPre, Shoot2, Shoot3, Shoot4,
            IntkSt1, IntkSt2, IntkSt3, IntkFi1, IntkFi2, IntkFi3;

    public void buildPaths() {

        OpenGate = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal1, GateOpen))
                .setLinearHeadingInterpolation(IntkFinal1.getHeading(), GateOpen.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        ShootPre = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal1, Shoot))
                .setLinearHeadingInterpolation(IntkFinal1.getHeading(), Shoot.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(IntkFinal2, ControlIntk2, Shoot))
                .setLinearHeadingInterpolation(IntkFinal2.getHeading(), Shoot.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal3, FinalShootPose))
                .setLinearHeadingInterpolation(IntkFinal3.getHeading(), FinalShootPose.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        IntkSt1 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart1))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart1.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        IntkFi1 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart1, IntkFinal1))
                .setLinearHeadingInterpolation(IntkStart1.getHeading(), IntkFinal1.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        IntkSt2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart2))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart2.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        IntkFi2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart2, IntkFinal2))
                .setLinearHeadingInterpolation(IntkStart2.getHeading(), IntkFinal2.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        IntkSt3 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart3))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart3.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        IntkFi3 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart3, IntkFinal3))
                .setLinearHeadingInterpolation(IntkStart3.getHeading(), IntkFinal3.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
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
            case 1:
                indexer_pgp.startPreSpin();
                setPathState(2);
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(ShootPre, 1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    indexer_pgp.startLine4Outtake();
                    setPathState(4);
                }
                break;

            case 4:
                if (!modeLocked) {
                    int id = getAprilTagID();

                    if (id == 21) {
                        lockModeTo(indexer_gpp, "GPP", id);
                        setPathState(5);
                    } else if (id == 22) {
                        lockModeTo(indexer_pgp, "PGP", id);
                        setPathState(5);
                    } else if (id == 23) {
                        lockModeTo(indexer_ppg, "PPG", id);
                        setPathState(5);
                    } else if (pathTimer.getElapsedTimeSeconds() >= 1) {
                        lockModeTo(indexer_noSort, "NoSort", -1);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    startIntakeLine(1);
                    follower.followPath(IntkSt1, 1, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi1, intakeFinalSpeedForLine(1), true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(OpenGate, 1, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    indexer_pgp.startPreSpin();
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, 1, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    startOuttakeLine(1);
                    setPathState(11);
                }
                break;

            case 11:
                if (!activeIndexerBusy()) {
                    follower.followPath(IntkSt2, 1, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    startIntakeLine(2);
                    follower.followPath(IntkFi2, intakeFinalSpeedForLine(2), true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    indexer_pgp.startPreSpin();
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy() ) {
                    follower.followPath(Shoot3, 1, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    startOuttakeLine(2);
                    setPathState(16);
                }
                break;

            case 16:
                if (!activeIndexerBusy()) {
                    follower.followPath(IntkSt3, 1, true);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    startIntakeLine(3);
                    indexer_pgp.startPreSpin();
                    follower.followPath(IntkFi3, intakeFinalSpeedForLine(3), true);
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, 1, true);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    startOuttakeLine(3);
                    setPathState(20);
                }
                break;

            case 20:
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
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
        if (actionTimer != null) actionTimer.resetTimer();
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

        indexer_pgp.startPreSpin();

        activeIndexer = null;
        setPathState(1);
    }

    @Override
    public void init_loop() {
        if (indexer_pgp != null) {
            Pose pose = follower.getPose();
            double distance = getDistanceToGoal();
            indexer_pgp.setShootContext(pose.getX(), pose.getY(), distance);
            indexer_pgp.update();
        } else if (Shooter != null) {
            Pose pose = follower.getPose();
            double distance = getDistanceToGoal();
            Shooter.updateShootingAuto(false, pose.getX(), pose.getY(), distance);
            Shooter.update();
        }
    }

    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.getPose();
        double distance = getDistanceToGoal();

        if (activeIndexer != null) {
            activeIndexer.setShootContext(pose.getX(), pose.getY(), distance);
            activeIndexer.update();
        } else if (indexer_pgp != null) {
            indexer_pgp.setShootContext(pose.getX(), pose.getY(), distance);
            indexer_pgp.update();
        } else if (Shooter != null) {
            Shooter.updateShootingAuto(false, pose.getX(), pose.getY(), distance);
            Shooter.update();
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

    public double getDistanceToGoal() {
        double gx = 0;
        double gy = 140;
        double dx = gx - follower.getPose().getX();
        double dy = gy - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }
}