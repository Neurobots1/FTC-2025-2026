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
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Launcher23511;
import org.firstinspires.ftc.teamcode.SubSystem.Auto_pathBuild_Blue;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_GPP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PPG;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PGP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Autonomous(name = "Auto_blue1", group = "Examples")
public class Auto_Blue1 extends OpMode {

    private Follower follower;
    private Auto_pathBuild_Blue autoPathBuild;
    private RevColorSensorV3 colorSensor;
    private AprilTagPipeline aprilTag;
    private Robot robot;

    private Indexer_Base indexer;
    private Indexer_GPP indexer_gpp;
    private Indexer_PGP indexer_pgp;
    private Indexer_PPG indexer_ppg;

    private IntakeMotor intkM;

    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;

    public static boolean usePIDF = true;
    public static double targetTicksPerSecond = 725;
    public static double rawPower = -0.5;
    public static boolean rawPowerMode = false;

    private VoltageSensor voltageSensor;
    private Launcher23511 Shooter;
    public static boolean shooterEnabled = false;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private TelemetryManager telemetryM;

    private boolean useGPPMode = false;
    private boolean usePGPMode = false;
    private boolean usePPGMode = false;
    private boolean colorSensorActivate = false;

    // on lock le mode une fois
    private boolean modeLocked = false;
    boolean AutoShoot = false;

    public final Pose startPose = new Pose(33, 136, Math.toRadians(88));
    private final Pose SeePatern = new Pose(52, 95, Math.toRadians(70));
    private final Pose Shoot = new Pose(49, 95, Math.toRadians(136));
    private final Pose IntkStart1 = new Pose(50, 85, Math.toRadians(180));
    private final Pose IntkFinal1 = new Pose(16, 85, Math.toRadians(180));
    private final Pose IntkStart2 = new Pose(50, 63, Math.toRadians(180));
    private final Pose IntkFinal2 = new Pose(10, 63, Math.toRadians(180));
    private final Pose IntkStart3 = new Pose(50, 40, Math.toRadians(180));
    private final Pose IntkFinal3 = new Pose(7, 37.05, Math.toRadians(180));
    private final Pose FinalShootPose = new Pose(55, 105, Math.toRadians(143));

    public static Pose finalPose = new Pose(0, 0, 0);

    public PathChain TakePatern, Shoot1, Shoot2, Shoot3, Shoot4,
            IntkSt1, IntkSt2, IntkSt3, IntkFi1, IntkFi2, IntkFi3, FiString;

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

        FiString = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal3, FinalShootPose))
                .setLinearHeadingInterpolation(IntkFinal3.getHeading(), FinalShootPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // démarre TakePatern une fois
                if (!follower.isBusy()) {
                    follower.followPath(TakePatern, 1, true);
                    setPathState(1);
                }
                break;

            case 1:
                // Attends un peu pour laisser la cam détecter, puis lock le mode une fois
                if (!modeLocked && pathTimer.getElapsedTimeSeconds() >= 3.0) {
                    int id = getAprilTagID();

                    if (id == 21) {
                        useGPPMode = true;
                        telemetry.addData("Mode", "GPP Mode Activé - ID 21 détecté");
                    } else if (id == 22) {
                        usePGPMode = true;
                        telemetry.addData("Mode", "PGP Mode Activé - ID 22 détecté");
                    } else if (id == 23) {
                        usePPGMode = true;
                        telemetry.addData("Mode", "PPG Mode Activé - ID 23 détecté");
                    } else {
                        // aucun tag -> reste "normal" (aucun mode)
                        telemetry.addData("Mode", "Normal (aucun tag détecté)");
                    }

                    modeLocked = true;
                    setPathState(40);
                }
                break;

            case 40:

                // sinon followPath peut être ignoré si follower est busy
                if (!follower.isBusy()) {
                    follower.followPath(Shoot1, 1, true);
                    shooterEnabled = true;


                    setPathState(3000);
                }
                break;

            case 3000:
                // Attends que le robot soit posé et que le flywheel soit ready
                // (tu avais un >=3, je le garde)
                if (!follower.isBusy()) {
                    AutoShoot = true;


                    if (Shooter.flywheelReady()) {
                        intkM.intake();
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2) {
                    AutoShoot = false;
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(IntkSt1, 1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    indexer.IndexBlocker();
                    intkM.slowIntake();
                    follower.followPath(IntkFi1, 0.3, true);


                    if (useGPPMode && indexer_gpp != null && colorSensor.getDistance(DistanceUnit.MM) <= 80) {
                        intkM.intake();
                        indexer_gpp.startLine1();
                        setPathState(5);
                    } else if (usePGPMode && indexer_pgp != null && colorSensor.getDistance(DistanceUnit.MM) <= 80) {
                        intkM.intake();
                        indexer.indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                        indexer_pgp.startLine1Intake();
                        setPathState(5);
                    } else if (usePPGMode && colorSensor.getDistance(DistanceUnit.MM) <= 80) {
                        intkM.intake();
                        setPathState(5);
                    }


                }
                break;

            case 5:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 5)  {
                    follower.followPath(Shoot2, 1, true);
                    setPathState(6);


            }


                break;

            case 6:
                if (!follower.isBusy()) {
                    AutoShoot = true;

                    if (Shooter != null && Shooter.flywheelReady() && !follower.isBusy()) {
                        setPathState(68);
                    }
                }
                break;

            case 68:
                intkM.intake();
                indexer_pgp.startLine1Outtake();
                setPathState(4000);

                break;

            case 4000:

                if (actionTimer.getElapsedTimeSeconds() > 6) {
                    setPathState(-1);
                }

                break;

            case 7:
                if (!follower.isBusy()) {
                    AutoShoot = false;
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(IntkSt2, 1, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi2, 0.7, true);

                    if (useGPPMode && indexer_gpp != null) {
                        indexer_gpp.startLine2();
                    } else if (usePGPMode) {
                        intkM.intake();
                    } else if (usePPGMode && indexer_ppg != null) {
                        indexer_ppg.startLine2();
                    }

                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot3, 1, true);
                    setPathState(12);
                }

               else if (!follower.isBusy()) {
                    follower.followPath(Shoot3, 1, true);
                    setPathState(12);
                }

               else if (!follower.isBusy()) {
                    follower.followPath(Shoot3, 1, true);
                    setPathState(12);
                }

                break;

            case 12:
                if (!follower.isBusy()) {
                    AutoShoot = true;
                    intkM.slowOuttake();


                    if (Shooter != null && Shooter.flywheelReady()) {
                        setPathState(69);
                    }
                }
                break;

            case 69:
                intkM.intake();
                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    AutoShoot = false;
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(IntkSt3, 1, true);
                    intkM.intake();
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi3, 0.7, true);
                    indexer.IndexBlocker();

                    if (useGPPMode) {
                        intkM.intake();
                    } else if (usePGPMode && indexer_pgp != null) {
                        indexer_pgp.startLine3Intake();
                    } else if (usePPGMode && indexer_ppg != null) {
                        indexer_ppg.startLine3();
                    }

                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, 1, true);
                    setPathState(17);
                }

                if (!follower.isBusy() && indexer_pgp.isBusy()) {
                    follower.followPath(Shoot4, 1, true);
                    setPathState(17);
                }

                if (!follower.isBusy() && indexer_ppg.isBusy()) {
                    follower.followPath(Shoot4, 1, true);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    intkM.slowOuttake();
                    AutoShoot = true;

                    if (Shooter != null && Shooter.flywheelReady() && !follower.isBusy()) {
                        setPathState(70);
                    }
                }
                break;

            case 70:
                if (!follower.isBusy()) {
                    intkM.intake();
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
                        setPathState(18);
                    }
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    AutoShoot = false;
                    setPathState(-1);
                }
                break;

            case -1:
                // idle
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

        if (detections != null && !detections.isEmpty()) {
            return detections.get(0).id;
        }
        return -1;
    }

    @Override
    public void stop(){

    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
        if (actionTimer != null) actionTimer.resetTimer();
    }

    @Override
    public void loop() {

        Shooter.update();
        if (colorSensor.getDistance(DistanceUnit.MM) <= 70){
            colorSensorActivate = true;
        }

        Pose pose = follower.getPose();
        double distance = getDistanceToGoal();
        Shooter.updateShooting(AutoShoot, pose.getX(), pose.getY(), distance);
        follower.update();

        if (useGPPMode && indexer_gpp != null) {
            indexer_gpp.Line1();
            indexer_gpp.Line2();
        }

        if (usePGPMode && indexer_pgp != null) {
            indexer_pgp.Line1Intake();
            indexer_pgp.Line3Intake();
            indexer_pgp.Line1Outtake();
        }

        if (usePPGMode && indexer_ppg != null) {
            indexer_ppg.Line2();
            indexer_ppg.Line3();
        }



        // si ton Launcher a un update(), tu peux le mettre; sinon enlève
        // (je ne l’ai pas forcé ici pour ne pas casser compilation)
        // if (Shooter != null) Shooter.update();

        autonomousPathUpdate();

        telemetry.addData("Mode", useGPPMode ? "GPP" : (usePGPMode ? "PGP" : (usePPGMode ? "PPG" : "Normal")));
        telemetry.addData("ModeLocked", modeLocked);
        telemetry.addData("AprilTag ID", getAprilTagID());
        telemetry.addData("path state", pathState);
        telemetry.addData("followerBusy", follower.isBusy());
        telemetry.addData("pathTimer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("actionTimer", actionTimer.getElapsedTimeSeconds());

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.addData("shooterEnabled", shooterEnabled);
        telemetry.addData("usePIDF", usePIDF);
        telemetry.addData("rawPowerMode", rawPowerMode);
        telemetry.addData("targetTicksPerSecond", targetTicksPerSecond);
        telemetry.addData("GPP State indxer", indexer_gpp.gppState1);
        telemetry.addData("ColorSensorActivate", colorSensor);

        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();


        indexer = new Indexer_Base(hardwareMap);
        indexer_gpp = new Indexer_GPP(hardwareMap, indexer);
        indexer_pgp = new Indexer_PGP(hardwareMap, indexer);
        indexer_ppg = new Indexer_PPG(hardwareMap, indexer);
        indexer.IndexBlocker();

        robot = new Robot();

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        telemetry.addData("Status", "Initialized");

        aprilTag = new AprilTagPipeline(hardwareMap);
        aprilTag.startCamera();

        intkM = new IntakeMotor(hardwareMap);

        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();



        Shooter = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        Servo blocker = hardwareMap.get(Servo.class, "Blocker");
        Shooter.setBlocker(blocker);
        Shooter.setFlywheelTicks(725);
        shooterEnabled = false;
        Shooter.init();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        autoPathBuild = new Auto_pathBuild_Blue(follower);
        follower.setStartingPose(startPose);

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
