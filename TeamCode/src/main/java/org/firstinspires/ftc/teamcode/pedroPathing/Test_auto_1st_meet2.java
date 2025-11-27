package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shoot;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.SubSystem.Servo_HOOD;

@Autonomous(name = "Auto_2", group = "Examples")
public class Test_auto_1st_meet2 extends OpMode {

    private Follower follower;
    private HardwareMap hardwareMap;
  //NIGGERfarm(cotoneslave)  private IntakeMotor intkM;
    private Shoot shootM;
    private Servo_HOOD servo;

    private Robot robot;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;



    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose ShootPose = new Pose(67, 18, Math.toRadians(139));
    private final Pose IntkStart1 = new Pose(50, 90, Math.toRadians(180));
    private final Pose IntkFinal1 = new Pose(30, 90, Math.toRadians(180));
    private final Pose IntkStart2 = new Pose(50, 65, Math.toRadians(180));
    private final Pose IntkFinal2 = new Pose(30, 65, Math.toRadians(180));
    private final Pose IntkStart3 = new Pose(50, 50, Math.toRadians(180));
    private final Pose IntkFinal3 = new Pose(30, 50, Math.toRadians(180));
    private final Pose FinalPose = new Pose(67, 60, Math.toRadians(135));

    private Path b;

    private PathChain Shoot1,Shoot2,Shoot3,Shoot4,IntkSt1,IntkSt2,IntkSt3,IntkFi1,IntkFi2,IntkFi3,FiString;
    public void buildPaths() {

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading() , ShootPose.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal1, ShootPose))
                .setLinearHeadingInterpolation(IntkFinal1.getHeading() , ShootPose.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal2, ShootPose))
                .setLinearHeadingInterpolation(IntkFinal2.getHeading() , ShootPose.getHeading())
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal3, ShootPose))
                .setLinearHeadingInterpolation(IntkFinal3.getHeading() , ShootPose.getHeading())
                .build();


        IntkSt1 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, IntkStart1))
                .setLinearHeadingInterpolation(ShootPose.getHeading() , IntkStart1.getHeading())
                .build();

        IntkFi1 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart1, IntkFinal1))
                .setLinearHeadingInterpolation(IntkStart1.getHeading() , IntkFinal1.getHeading())
                .build();



        IntkSt2 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, IntkStart2))
                .setLinearHeadingInterpolation(ShootPose.getHeading() ,IntkStart2.getHeading())
                .build();

        IntkFi2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart2, IntkFinal2))
                .setLinearHeadingInterpolation(IntkStart2.getHeading() ,IntkFinal2.getHeading())
                .build();



        IntkSt3 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, IntkStart3))
                .setLinearHeadingInterpolation(ShootPose.getHeading() ,IntkStart3.getHeading())
                .build();

        IntkFi3 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart3, IntkFinal3))
                .setLinearHeadingInterpolation(IntkStart3.getHeading() ,IntkFinal3.getHeading())
                .build();


        FiString = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, FinalPose))
                .setLinearHeadingInterpolation(ShootPose.getHeading() ,FinalPose.getHeading())
                .build();




    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Shoot1, 0.7, true);

                if (!follower.isBusy()) {
                    //shootM.();
                }
                setPathState(1);


                break;


            case 1:
                if (!follower.isBusy()) {
                    //shootM.();
  //                  intkM.();

                    follower.followPath(IntkSt1, 0.7, true);
                    setPathState(2);

                }
                break;


            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi1, 0.7, true);

                    if (!follower.isBusy()) {
    //                    intkM.();
                    }
                    setPathState(3);

                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, 0.7, true);

                    if (!follower.isBusy()){
                        //shootM.();
                    }
                    setPathState(4);
                }
                break;

            case 4:
                if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(5);

                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(IntkSt2, 0.7, true);

                    if (!follower.isBusy()) {
      //                  intkM.();

                    }
                    setPathState(6);




                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi2, 0.7, true);

                    if (!follower.isBusy()) {
        //                intkM.();
                    }
                    setPathState(7);



                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot3, 0.7, true);

                    if (!follower.isBusy()) {
                       // shootM.();
                    }
                    setPathState(8);





                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(IntkSt3, 0.7, true);

                    if (!follower.isBusy()) {
          //              intkM.();
                    }
                    setPathState(9);




                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi3, 0.7, true);

                    if (!follower.isBusy()) {
            //            intkM.();
                    }
                    setPathState(10);




                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, 0.7, true);

                    if (!follower.isBusy()) {
                        //shootM.();
                    }
                    setPathState(11);




                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(FiString, 0.7, true);

                    if (!follower.isBusy()) {
                        //shootM.();
                       servo.getCurrentAngle();
                       servo.setAngle(90);
                    }
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


    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();


    }

    /** This method is called once at the setup of the OpMode. **/
    @Override
    public void init() {

//        intkM = new IntakeMotor(hardwareMap);




        buildPaths();
    }
    @Override
    public void start() {
        setPathState(0);
    }
}
