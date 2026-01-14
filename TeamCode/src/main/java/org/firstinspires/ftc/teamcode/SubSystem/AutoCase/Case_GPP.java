package org.firstinspires.ftc.teamcode.SubSystem.AutoCase;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.SubSystem.Auto_pathBuild_Blue;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Shoot;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Launcher23511;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipelinetele;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_GPP;

public class Case_GPP {
    public Indexer_Base indexerBase;
    public IntakeMotor intkM;
    public Robot robot;

    private Follower follower;
    private Auto_pathBuild_Blue autoPathBuild;
    AprilTagPipelinetele aprilTag;
    private DcMotorEx flywheelMotorOne;
    private  DcMotorEx flywheelMotorTwo;
    public static boolean usePIDF = true;
    public static double targetTicksPerSecond = 725;
    public static double rawPower = -0.5;
    public static boolean rawPowerMode = false;
    public Indexer_GPP indexGPP;
    private VoltageSensor voltageSensor;
    //private Shoot shootM;
    private Launcher23511 Shooter;
    public static boolean shooterEnabled = false;
    private Tuning tuning;
    private Robot init;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private TelemetryManager telemetryM;
    private Shoot ShootM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;




    public void autonomousPathUpdate() {  //Change the cases because of the indexer
        switch (pathState) {

            case 0:
                follower.followPath(autoPathBuild.Shoot1, 1, true);
                shooterEnabled = true;
                setPathState(1);

                break;

            case 1:
                if (!follower.isBusy()) {
                    intkM.slowOuttake();
                    usePIDF = true;
                    rawPowerMode = false;
                    //shooterEnabled = true;
                    if (Shooter.flywheelReady() && !follower.isBusy()) {
                        setPathState(67);
                    }



                }
                break;

            case 67:
                intkM.intake();
                if (actionTimer.getElapsedTimeSeconds() > 2   ) {
                    setPathState(2);
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
                    indexGPP.Line1();
                    if (!follower.isBusy()) {

                        setPathState(5);
                    }
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
                   indexGPP.Line2();
                    if (actionTimer.getElapsedTimeSeconds() > 1.5) {

                        setPathState(11);
                    }


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
                    if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                        setPathState(16);
                    }
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
                if (!follower.isBusy()){
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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }
}

