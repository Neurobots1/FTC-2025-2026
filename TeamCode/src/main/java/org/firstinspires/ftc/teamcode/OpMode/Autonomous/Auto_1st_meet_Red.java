package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shoot;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Launcher23511;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.HeadingLockOJB;

@Autonomous(name = "Auto_1st_meet_red", group = "Examples")
public class Auto_1st_meet_Red extends OpMode {

    private Follower follower;

    private IntakeMotor intkM;
    private  DcMotorEx flywheelMotorOne;
    private  DcMotorEx flywheelMotorTwo;
    public static boolean usePIDF = true;
    public static double targetTicksPerSecond = 785;
    public static double rawPower = -1;
    public static boolean rawPowerMode = false;
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

    private final Pose startPose = new Pose(110, 136, Math.toRadians(90));
    private final Pose Shoot = new Pose(92, 92, Math.toRadians(50));
    private final Pose IntkStart1 = new Pose(100, 85, Math.toRadians(0));
    private final Pose IntkFinal1 = new Pose(127, 85, Math.toRadians(0));
    private final Pose IntkStart2 = new Pose(100, 63, Math.toRadians(0));
    private final Pose IntkFinal2 = new Pose(127, 63, Math.toRadians(0));
    private final Pose IntkStart3 = new Pose(100, 40, Math.toRadians(0));
    private final Pose IntkFinal3 = new Pose(127, 40, Math.toRadians(0));
    private final Pose FinalShootPose = new Pose(93, 108, Math.toRadians(37));
    public static Pose finalPose = new Pose();


    private Path a;

    private PathChain Shoot1, Shoot2, Shoot3, Shoot4, IntkSt1, IntkSt2, IntkSt3, IntkFi1, IntkFi2, IntkFi3, FiString;

    public void buildPaths() {

        // Shoot PathBuild 1-4
        // Intake PathBuild 5-10
        //Final Pose PathBuild 11

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
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
                follower.followPath(Shoot1, 1, true);
                shooterEnabled = true;
                setPathState(1);

                break;

            case 1:
                if (!follower.isBusy()) {
                    intkM.slowOuttake();
                    usePIDF = true;
                    rawPowerMode = false;
                    //shooterEnabled = true;
                    if (Shooter.flywheelReady()) {
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
                    follower.followPath(IntkSt1, 1, true);
                    setPathState(4);
                }

                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi1, 0.5, true);
                    if (actionTimer.getElapsedTimeSeconds() > 1.5) {

                        setPathState(5);
                    }
                }
                break;


            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, 1, true);
                    setPathState(6);


                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intkM.slowOuttake();
                    rawPowerMode = false;
                    usePIDF = true;
                    if (Shooter.flywheelReady()) {
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
                    follower.followPath(IntkSt2, 1, true);


                    setPathState(10);


                }
                break;


            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi2, 0.7, true);
                    if (actionTimer.getElapsedTimeSeconds() > 1.5) {

                        setPathState(11);
                    }


                }
                break;


            case 11:
                if (!follower.isBusy()) {

                    follower.followPath(Shoot3, 1, true);
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
                    follower.followPath(IntkSt3, 1, true);

                    setPathState(15);

                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi3, 0.7, true);
                    if (actionTimer.getElapsedTimeSeconds() > 1.5) {

                        setPathState(16);
                    }
                }
                break;


            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, 1, true);
                    setPathState(17);


                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    intkM.slowOuttake();
                    rawPowerMode = false;
                    usePIDF = true;
                    //shooterEnabled = true;
                    if (Shooter.flywheelReady()) {
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


    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        if (rawPowerMode) {
            flywheelMotorOne.setPower(rawPower);
            flywheelMotorTwo.setPower(rawPower);
        } else {
            if (shooterEnabled) {
                Shooter.setFlywheelTicks(targetTicksPerSecond);
            } else {
                Shooter.setFlywheelTicks(0);
            }
            Shooter.update();
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooterEnabled", shooterEnabled);
        telemetry.addData("usePIDF", usePIDF);
        telemetry.addData("targetTicksPerSecond", targetTicksPerSecond);
        telemetry.update();


    }

    /**
     * This method is called once at the setup of the OpMode.
     **/
    @Override
    public void init() {
        intkM = new IntakeMotor(hardwareMap);
        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        Shooter = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        Shooter.setFlywheelTicks(785);
        shooterEnabled = false;

        Shooter.init();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        init = new Robot(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        //shooterLeft = hardwareMap.get(DcMotorEx.class, "ShooterA");
        //shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterB");
        //shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        follower.update();


        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop(){
        finalPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
    }
}