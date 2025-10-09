package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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

import java.util.Set;

@Autonomous(name = "Test_pedro", group = "Examples")
public class Test_pedro extends OpMode {

    private Follower follower;
    private Init init;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    


    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private final Pose startPose = new Pose(63, 136, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose ShootingPose = new Pose(52, 92, Math.toRadians(139));
    private final Pose Intake1Pose = new Pose(20,59, Math.toRadians(180));
    private final Pose GatePose = new Pose(15,63,Math.toRadians(180));
    private final Pose Intake2Start = new Pose(49,36,Math.toRadians(180));
    private final Pose Intake2Final = new Pose(18,36,Math.toRadians(180));
    private final Pose Intake3Start = new Pose(42,84,Math.toRadians(180));
    private final Pose Intake3Final = new Pose(20,84,Math.toRadians(180));
    private final Pose ControlPointIntk1 = new Pose(86,59);
    private final Pose ControlPointShoot2 = new Pose(62,67);
    private final Pose ControlPointShoot3 = new Pose(68,72);


    private Path d;
    private PathChain Shoot1,Intake1,Gate,Shoot2,Intake2Pose,Intake2Line,Shoot3,Intake3Pose,Intake3Line,Shoot4;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading() , ShootingPose.getHeading())
                .build();

        Intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootingPose, ControlPointIntk1, Intake1Pose))
                .setLinearHeadingInterpolation(ShootingPose.getHeading() , Intake1Pose.getHeading())
                .build();

        Gate = follower.pathBuilder()
                .addPath(new BezierLine(Intake1Pose, GatePose))
                .setLinearHeadingInterpolation(Intake1Pose.getHeading() , GatePose.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(GatePose, ControlPointShoot2, ShootingPose))
                .setLinearHeadingInterpolation(GatePose.getHeading() , ShootingPose.getHeading())
                .build();

        Intake2Pose = follower.pathBuilder()
                .addPath(new BezierLine(ShootingPose, Intake2Start))
                .setLinearHeadingInterpolation(ShootingPose.getHeading() , Intake2Start.getHeading())
                .build();

        Intake2Line = follower.pathBuilder()
                .addPath(new BezierLine(Intake2Start, Intake2Final))
                .setLinearHeadingInterpolation(Intake2Start.getHeading() , Intake2Final.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(Intake2Final,ControlPointShoot3, ShootingPose))
                .setLinearHeadingInterpolation(Intake2Final.getHeading() , ShootingPose.getHeading())
                .build();

        Intake3Pose = follower.pathBuilder()
                .addPath(new BezierLine(ShootingPose, Intake3Start))
                .setLinearHeadingInterpolation(ShootingPose.getHeading() , Intake3Start.getHeading())
                .build();

        Intake3Line = follower.pathBuilder()
                .addPath(new BezierLine(Intake3Start, Intake3Final))
                .setLinearHeadingInterpolation(Intake3Start.getHeading() , Intake3Final.getHeading())
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(Intake3Final, ShootingPose))
                .setLinearHeadingInterpolation(Intake3Final.getHeading() , ShootingPose.getHeading())
                .build();

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Shoot1,1,true);
                setPathState(1);
                break;


            case 1:
                if (!follower.isBusy()){
                    follower.followPath(Intake1,1,true);
                    setPathState(2);

                }
                break;
            case 2:
                if (!follower.isBusy()){
                    follower.followPath(Gate,1,true);
                    setPathState(3);

                }
                break;

            case 3:
                if (!follower.isBusy()){
                    follower.followPath(Shoot2,1,true);
                    setPathState(4);

                }
                break;

            case 4:
                if (actionTimer.getElapsedTimeSeconds()>0.5){
                    setPathState(5);

                }
                break;

            case 5:
                if (!follower.isBusy()){
                    follower.followPath(Intake2Pose,1,true);
                    setPathState(6);

                }
                break;

            case 6:
                if (!follower.isBusy()){
                    follower.followPath(Intake2Line,1,true);
                    setPathState(7);

                }
                break;

            case 7:
                if (!follower.isBusy()){
                    follower.followPath(Shoot3,1,true);
                    setPathState(8);

                }
                break;

            case 8:
                if (actionTimer.getElapsedTimeSeconds()>0.5){
                    setPathState(9);

                }
                break;

            case 9:
                if (!follower.isBusy()){
                    follower.followPath(Intake3Pose,1,true);
                    setPathState(10);

                }
                break;

            case 10:
                if (!follower.isBusy()){
                    follower.followPath(Intake3Line,1,true);
                    setPathState(11);

                }
                break;
            case 11:
                if (!follower.isBusy()){
                    follower.followPath(Shoot4,1,true);
                    setPathState(12);
                }

            case 12:
                if (actionTimer.getElapsedTimeSeconds()>0.5){
                    setPathState(-1);

                }

                break;


        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
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
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        buildPaths();
    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}