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

@Autonomous(name = "Auto_blue2", group = "Examples")
public class Auto_Blue2 extends OpMode {

    private Follower follower;

    private IntakeMotor intkM;
    private Robot robot;
    private  DcMotorEx flywheelMotorOne;
    private  DcMotorEx flywheelMotorTwo;
    public static boolean usePIDF = true;
    public static double targetTicksPerSecond = 725;
    public static double rawPower = -0.5;
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

    private final Pose startPose = new Pose(33, 136, Math.toRadians(90));
    private final Pose Shoot = new Pose(56, 10, Math.toRadians(106));
    private final Pose IntkStart1 = new Pose(56, 36, Math.toRadians(180));
    private final Pose IntkFinal1 = new Pose(7, 36, Math.toRadians(180));
    private final Pose IntkStart2 = new Pose(56, 60, Math.toRadians(180));
    private final Pose IntkFinal2 = new Pose(7, 60, Math.toRadians(180));
    private final Pose IntkStart3 = new Pose(56, 85, Math.toRadians(180));
    private final Pose IntkFinal3 = new Pose(15, 85, Math.toRadians(180));
    public static Pose finalPose2 = new Pose(56,28, Math.toRadians(90));

    private Path a;

    private PathChain Shoot1, Shoot2, Shoot3, Shoot4, IntkSt1, IntkSt2, IntkSt3, IntkFi1, IntkFi2, IntkFi3, FiString;

    public void buildPaths() {

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
                .addPath(new BezierLine(IntkFinal3, Shoot))
                .setLinearHeadingInterpolation(IntkFinal3.getHeading(), Shoot.getHeading())
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


        Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal3, Shoot))
                .setLinearHeadingInterpolation(IntkFinal3.getHeading(), Shoot.getHeading())
                .build();

        FiString = follower.pathBuilder()
                .addPath(new BezierLine(Shoot,finalPose2))
                .setLinearHeadingInterpolation(Shoot.getHeading(), finalPose2.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            //put cases

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
        Shooter.setFlywheelTicks(725);
        shooterEnabled = false;

        Shooter.init();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        robot = new Robot();
        robot.init(hardwareMap);
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
        finalPose2 = new Pose(56,28, Math.toRadians(90));
    }
}
