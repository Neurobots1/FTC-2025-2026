package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.SubSystem.Init;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

import java.util.Set;

@Autonomous(name = "Test_auto_1st_meet", group = "Examples")
public class Test_auto_1st_meet extends OpMode {

    private Follower follower;
    private HardwareMap hardwareMap;
    private IntakeMotor intkM;
    private Tuning tuning;
    private Init init;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;



    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private final Pose startPose = new Pose(33, 136, Math.toRadians(90));
    private final Pose Shoot = new Pose(52, 92, Math.toRadians(139));
    private final Pose IntkStart1 = new Pose(50, 90, Math.toRadians(180));
    private final Pose IntkFinal1 = new Pose(30, 90, Math.toRadians(180));
    private final Pose IntkStart2 = new Pose(50, 65, Math.toRadians(180));
    private final Pose IntkFinal2 = new Pose(30, 65, Math.toRadians(180));
    private final Pose IntkStart3 = new Pose(50, 50, Math.toRadians(180));
    private final Pose IntkFinal3 = new Pose(30, 50, Math.toRadians(180));
    private final Pose FinalPose = new Pose(52, 80, Math.toRadians(135));

    private Path a;

    private PathChain Shoot1,Shoot2,Shoot3,Shoot4,IntkSt1,IntkSt2,IntkSt3,IntkFi1,IntkFi2,IntkFi3,FiString;
    public void buildPaths() {

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading() , Shoot.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal1,Shoot))
                .setLinearHeadingInterpolation(IntkFinal1.getHeading() , Shoot.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal2,Shoot))
                .setLinearHeadingInterpolation(IntkFinal2.getHeading() , Shoot.getHeading())
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal3,Shoot))
                .setLinearHeadingInterpolation(IntkFinal3.getHeading() , Shoot.getHeading())
                .build();


        IntkSt1 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart1))
                .setLinearHeadingInterpolation(Shoot.getHeading() , IntkStart1.getHeading())
                .build();

        IntkFi1 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart1, IntkFinal1))
                .setLinearHeadingInterpolation(IntkStart1.getHeading() , IntkFinal1.getHeading())
                .build();



        IntkSt2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart2))
                .setLinearHeadingInterpolation(Shoot.getHeading() ,IntkStart2.getHeading())
                .build();

        IntkFi2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart2, IntkFinal2))
                .setLinearHeadingInterpolation(IntkStart2.getHeading() ,IntkFinal2.getHeading())
                .build();



        IntkSt3 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart3))
                .setLinearHeadingInterpolation(Shoot.getHeading() ,IntkStart3.getHeading())
                .build();

        IntkFi3 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart3, IntkFinal3))
                .setLinearHeadingInterpolation(IntkStart3.getHeading() ,IntkFinal3.getHeading())
                .build();


        FiString = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, FinalPose))
                .setLinearHeadingInterpolation(Shoot.getHeading() ,FinalPose.getHeading())
                .build();




    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Shoot1, 0.7, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(IntkSt1, 0.7, true);
                    setPathState(2);

                }
                break;


            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi1, 0.7, true);
                    setPathState(3);

                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, 0.7, true);
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
                    setPathState(6);




                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi2, 0.7, true);
                    setPathState(7);




                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    intkM.intake();{}{}{{}{}{}{}{}{{{{}}}}}
                    follower.followPath(Shoot3, 0.7, true);
                    setPathState(8);




                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(IntkSt3, 0.7, true);
                    setPathState(9);




                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi3, 0.7, true);
                    setPathState(10);




                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, 0.7, true);
                    setPathState(11);




                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(FiString, 0.7, true);
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
        pathTimer = new Timer();
        intkM.hardware(hardwareMap);
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();




        buildPaths();
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}