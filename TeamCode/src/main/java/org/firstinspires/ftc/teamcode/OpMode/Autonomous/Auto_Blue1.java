package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

import java.util.List;

@Autonomous(name = "Auto_blue1", group = "Examples")
public class Auto_Blue1 extends OpMode {

    private Follower follower;
    private Auto_pathBuild_Blue autoPathBuild;
    private AprilTagPipeline aprilTag;
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
    private boolean gppModeChecked = false;

    private boolean usePGPMode = false;
    private boolean pgpModeChecked = false;

    private boolean usePPGMode = false;
    private boolean ppgModeChecked = false;

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(autoPathBuild.TakePatern, 1, true);
                setPathState(1);

            case 1:


                if (!gppModeChecked && !useGPPMode) {
                    int detectedID = getAprilTagID();
                    if (detectedID == 21) {
                        useGPPMode = true;
                        telemetry.addData("Mode", "GPP Mode Activé - ID 21 détecté");
                    }
                    gppModeChecked = true;
                } else if (!pgpModeChecked && !usePGPMode) {
                    int detectedID = getAprilTagID();
                    if (detectedID == 22) {
                        usePGPMode = true;
                        telemetry.addData("Mode", "PGP Mode Activé - ID 22 détecté");
                    }
                    pgpModeChecked = true;
                } else if (!ppgModeChecked && !usePPGMode) {
                    int detectedID = getAprilTagID();
                    if (detectedID == 23) {
                        usePPGMode = true;
                        telemetry.addData("Mode", "PPG Mode Activé - ID 23 détecté");
                    }
                    ppgModeChecked = true;


                }
                setPathState(2000);

            case 2000:
                follower.followPath(autoPathBuild.Shoot1, 1, true);
                shooterEnabled = true;
                setPathState(3000);





                break;

            case 3000:
                if (!follower.isBusy()) {
                    intkM.slowOuttake();
                    usePIDF = true;
                    rawPowerMode = false;
                    //shooterEnabled = true;
                    if (Shooter.flywheelReady() && !follower.isBusy()) {
                        setPathState(2);
                    }


                }
                break;



            case 2:
                if (!follower.isBusy()) {
                    //shooterEnabled = false;
                    usePIDF = false;
                    rawPowerMode = true;
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(autoPathBuild.IntkSt1, 1, true);


                    setPathState(4);
                }

                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(autoPathBuild.IntkFi1, 0.5, true);
                   if (useGPPMode){
                       indexer_gpp.startLine1();


                   } else if (usePGPMode) {
                       indexer_pgp.startLine1();


                   } else if (usePPGMode) {
                       intkM.intake();
                   }

                    setPathState(5);

                }
                break;


            case 5:
                follower.followPath(autoPathBuild.Shoot2, 1, true);
                if (!follower.isBusy()) {
                    setPathState(6);
                }


                break;

            case 6:
                if (!follower.isBusy()) {
                    intkM.slowOuttake();
                    rawPowerMode = false;
                    usePIDF = true;
                    if (Shooter.flywheelReady() && !follower.isBusy()) {
                        setPathState(68);
                    }
                }
                break;

            case 68:
                intkM.intake();
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    usePIDF = false;
                    rawPowerMode = true;
                    //shooterEnabled = false;
                    setPathState(9);
                }
                break;


            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(autoPathBuild.IntkSt2, 1, true);


                    setPathState(10);


                }
                break;


            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(autoPathBuild.IntkFi2, 0.7, true);
                    if (useGPPMode) {
                        indexer_gpp.startLine2();


                    } else if (usePGPMode) {
                        intkM.intake();


                    } else if (usePPGMode) {
                        indexer_ppg.startLine2();
                    }

                    setPathState(11);


                }
                break;


            case 11:
                if (!follower.isBusy()) {

                    follower.followPath(autoPathBuild.Shoot3, 1, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {

                    intkM.slowOuttake();
                    rawPowerMode = false;
                    usePIDF = true;
                    //shooterEnabled = true;
                    if (Shooter.flywheelReady()) {
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
                    usePIDF = false;
                    rawPowerMode = true;
                    //shooterEnabled = false;
                    setPathState(14);
                }

                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(autoPathBuild.IntkSt3, 1, true);
                    intkM.intake();
                    setPathState(15);

                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(autoPathBuild.IntkFi3, 0.7, true);
                    if (useGPPMode) {
                        intkM.intake();


                    } else if (usePGPMode) {
                        indexer_pgp.startLine3();


                    } else if (usePPGMode) {
                        indexer_ppg.startLine3();
                    }
                    setPathState(16);
                }
                break;


            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(autoPathBuild.Shoot4, 1, true);
                    setPathState(17);


                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    intkM.slowOuttake();
                    rawPowerMode = false;
                    usePIDF = true;
                    //shooterEnabled = true;
                    if (Shooter.flywheelReady() && !follower.isBusy()) {
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
                    usePIDF = false;
                    rawPowerMode = true;
                    //shooterEnabled = false;
                    setPathState(-1);
                }

                break;


        }

    }


    private int getAprilTagID() {
        List<AprilTagDetection> detections = aprilTag.getAllDetections();
        if (detections != null && !detections.isEmpty()) {
            return detections.get(0).id;
        }
        return -1;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Télémétrie
        telemetry.addData("Mode GPP", useGPPMode ? "ACTIVÉ" : "Normal");
        telemetry.addData("AprilTag ID", getAprilTagID());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooterEnabled", shooterEnabled);
        telemetry.addData("usePIDF", usePIDF);
        telemetry.addData("targetTicksPerSecond", targetTicksPerSecond);
        telemetry.update();
    }

    @Override
    public void init() {
        setPathState(0);
        intkM = new IntakeMotor(hardwareMap);
        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Shooter = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        Shooter.setFlywheelTicks(725);
        shooterEnabled = false;
        Shooter.init();

        //aprilTag = new AprilTagPipeline();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        autoPathBuild = new Auto_pathBuild_Blue(follower);
        follower.setStartingPose(autoPathBuild.startPose);
        autoPathBuild.buildPaths();

    }
}
