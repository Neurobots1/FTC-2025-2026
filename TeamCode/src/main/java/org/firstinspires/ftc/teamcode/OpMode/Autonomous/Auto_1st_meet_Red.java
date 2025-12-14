package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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
import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shoot;

@Autonomous(name = "Auto_1st_meet_Red", group = "Examples")
public class Auto_1st_meet_Red extends OpMode {

    private Follower follower;

    private IntakeMotor intkM;
    //private Shoot shootM;
    private Tuning tuning;
    private Robot init;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;



    private TelemetryManager telemetryM;
    private Shoot ShootM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private final Pose startPose = new Pose(110, 136, Math.toRadians(90));
    private final Pose Shoot = new Pose(92, 92, Math.toRadians(50));
    private final Pose IntkStart1 = new Pose(100, 85, Math.toRadians(0));
    private final Pose IntkFinal1 = new Pose(127, 85, Math.toRadians(0));
    private final Pose IntkStart2 = new Pose(100, 63, Math.toRadians(0));
    private final Pose IntkFinal2 = new Pose(127, 63, Math.toRadians(0));
    private final Pose IntkStart3 = new Pose(100, 40, Math.toRadians(0));
    private final Pose IntkFinal3 = new Pose(127, 40, Math.toRadians(0));
    private final Pose FinalPose = new Pose(89, 70, Math.toRadians(50));


    private Path a;

    private PathChain Shoot1,Shoot2,Shoot3,Shoot4,IntkSt1,IntkSt2,IntkSt3,IntkFi1,IntkFi2,IntkFi3,FiString;
    public void buildPaths() {

        // Shoot PathBuild 1-4
        // Intake PathBuild 5-10
        //Final Pose PathBuild 11

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading() , Shoot.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal1,Shoot))
                .setLinearHeadingInterpolation(IntkFinal1.getHeading() , Shoot.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal2, Shoot))
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
                follower.followPath(Shoot1, 1, true);
                if (actionTimer.getElapsedTimeSeconds() > 4) {
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()){
                    // mettre shooter(premier shoot)
                    if (actionTimer.getElapsedTimeSeconds() > 4){
                        //arreter shooter
                        setPathState(2);
                    }

                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(IntkSt1, 1, true);
                    setPathState(3);
                }

                break;

            case 3:
                if (!follower.isBusy()){
                    intkM.intake();
                        setPathState(4);

                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi1, 0.5, true);
                    if (actionTimer.getElapsedTimeSeconds() > 4) {

                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()){
                    intkM.stop();
                        setPathState(6);

                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, 1, true);

                    if (actionTimer.getElapsedTimeSeconds() > 4){
                        setPathState(7);
                    }



                }
                break;

            case 7:
                if (!follower.isBusy()){
                    //mettre shooter(deuxieme shoot)
                    if (actionTimer.getElapsedTimeSeconds() > 4){
                        //arreter shooter
                        setPathState(8);
                    }
                }
                break;




            case 8:
                if (actionTimer.getElapsedTimeSeconds() > 0.5) {

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
                if (!follower.isBusy()){
                    intkM.intake();
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi2, 0.5, true);
                    if (actionTimer.getElapsedTimeSeconds() > 4) {

                        setPathState(12);
                    }


                }
                break;

            case 12:
                if (!follower.isBusy()){
                    intkM.stop();
                        setPathState(13);

                }
                break;

            case 13:
                if (!follower.isBusy()) {

                    follower.followPath(Shoot3, 1, true);
                    if (actionTimer.getElapsedTimeSeconds() > 4){
                        setPathState(14);
                    }
                }
                break;

            case 14:
                if (!follower.isBusy()){
                    //mettre shooter(troisieme shoot)
                    if (actionTimer.getElapsedTimeSeconds() > 4){
                        //arreter shooter
                        setPathState(15);
                    }
                }

            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(IntkSt3, 1, true);

                    setPathState(16);

                }
                break;

            case 16:
                if (!follower.isBusy()){
                    intkM.intake();
                        setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi3, 0.5, true);
                    if (actionTimer.getElapsedTimeSeconds() > 4) {

                        setPathState(18);
                    }
                }
                break;

            case 18:
                if (!follower.isBusy()){
                    intkM.stop();
                        setPathState(19);
                }
                break;


            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, 1, true);
                    if (actionTimer.getElapsedTimeSeconds() > 4){
                        setPathState(20);
                    }


                }
                break;

            case 20:
                if (!follower.isBusy()){
                    //mettre shooter(quatrieme shoot)
                    if (actionTimer.getElapsedTimeSeconds() > 4){
                        //arreter shooter
                        setPathState(21);

                    }
                }

            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(FiString, 1, true);
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
        intkM = new IntakeMotor(hardwareMap);
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        init = new Robot(hardwareMap);
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