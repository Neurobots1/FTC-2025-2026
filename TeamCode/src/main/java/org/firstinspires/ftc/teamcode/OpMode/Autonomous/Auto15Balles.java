package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Rapid;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

@Autonomous(name = "Auto15Balles", group = "Examples")
public class Auto15Balles extends OpMode {

    private Follower follower;

    private Indexer_Base indexerBase;
    private Indexer_Rapid fuckingNoSort;
    private IntakeMotor intkM;

    private DcMotorEx flywheelMotorOne, flywheelMotorTwo;
    private VoltageSensor voltageSensor;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private LauncherSubsystem Shooter;

    public final Pose startPose = new Pose(20, 119, Math.toRadians(139));
    private final Pose shoot = new Pose(47, 92, Math.toRadians(139));
    private final Pose intkStart1 = new Pose(50, 88, Math.toRadians(190));
    private final Pose intkFinal1 = new Pose(23, 82, Math.toRadians(190));
    private final Pose intkStart2 = new Pose(50, 55, Math.toRadians(180));
    private final Pose intkFinal2 = new Pose(15, 55, Math.toRadians(180));
    private final Pose intk2Control = new Pose(45, 55);
    private final Pose gateIntk = new Pose(11, 55, Math.toRadians(152));
    private final Pose gateIntkControl = new Pose(27, 50);

    public PathChain Shoot1, IntkSt1, IntkFi1, Shoot2, IntkSt2, IntkFi2, Shoot3, GateIntk1, Shoot4, GateIntk2, FiShoot;

    private boolean gateWaitStarted1 = false;
    private boolean gateWaitStarted2 = false;

    public void buildPaths() {
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot.getHeading())
                .build();

        IntkSt1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot, intkStart1))
                .setLinearHeadingInterpolation(shoot.getHeading(), intkStart1.getHeading())
                .build();

        IntkFi1 = follower.pathBuilder()
                .addPath(new BezierLine(intkStart1, intkFinal1))
                .setLinearHeadingInterpolation(intkStart1.getHeading(), intkFinal1.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(intkFinal1, shoot))
                .setLinearHeadingInterpolation(intkFinal1.getHeading(), shoot.getHeading())
                .build();

        IntkSt2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot, intkStart2))
                .setLinearHeadingInterpolation(shoot.getHeading(), intkStart2.getHeading())
                .build();

        IntkFi2 = follower.pathBuilder()
                .addPath(new BezierLine(intkStart2, intkFinal2))
                .setLinearHeadingInterpolation(intkStart2.getHeading(), intkFinal2.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(intkFinal2, intk2Control, shoot))
                .setLinearHeadingInterpolation(intkFinal2.getHeading(), shoot.getHeading())
                .build();

        GateIntk1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, gateIntkControl, gateIntk))
                .setLinearHeadingInterpolation(shoot.getHeading(), gateIntk.getHeading())
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntk, intk2Control, shoot))
                .setLinearHeadingInterpolation(gateIntk.getHeading(), shoot.getHeading())
                .build();

        GateIntk2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, gateIntkControl, gateIntk))
                .setLinearHeadingInterpolation(shoot.getHeading(), gateIntk.getHeading())
                .build();

        FiShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntk, intk2Control, shoot))
                .setLinearHeadingInterpolation(gateIntk.getHeading(), shoot.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                fuckingNoSort.startSetupOuttake();
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot1, 0.8, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && Shooter.flywheelReady()) {
                    fuckingNoSort.startRapidOuttake();
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() && !fuckingNoSort.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(IntkSt2, 0.8, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi2, 0.6, true);
                    fuckingNoSort.startRapidIntakeNoSort();
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(Shoot3, 0.8, true);
                    intkM.stop();
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    fuckingNoSort.setShootContext(shoot.getX(), shoot.getY(), 77);
                    fuckingNoSort.startRapidOuttake();
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy() && !fuckingNoSort.isBusy()) {
                    follower.followPath(GateIntk1, 0.8, true);
                    gateWaitStarted1 = false;
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    if (!gateWaitStarted1) {
                        intkM.intake();
                        actionTimer.resetTimer();
                        gateWaitStarted1 = true;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
                        intkM.stop();
                        follower.followPath(Shoot4, 0.8, true);
                        gateWaitStarted1 = false;
                        setPathState(11);
                    }
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    intkM.stop();
                    fuckingNoSort.setShootContext(shoot.getX(), shoot.getY(), 77);
                    fuckingNoSort.startRapidOuttake();
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy() && !fuckingNoSort.isBusy()) {
                    follower.followPath(GateIntk2, 0.8, true);
                    gateWaitStarted2 = false;
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    if (!gateWaitStarted2) {
                        intkM.intake();
                        actionTimer.resetTimer();
                        gateWaitStarted2 = true;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
                        intkM.stop();
                        follower.followPath(FiShoot, 0.8, true);
                        gateWaitStarted2 = false;
                        setPathState(15);
                    }
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    intkM.stop();
                    fuckingNoSort.setShootContext(shoot.getX(), shoot.getY(), 77);
                    fuckingNoSort.startRapidOuttake();
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy() && !fuckingNoSort.isBusy()) {
                    follower.followPath(IntkSt1, 0.8, true);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi1, 0.8, true);
                    fuckingNoSort.startRapidIntakeNoSort();
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(Shoot2, 0.8, true);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    intkM.stop();
                    fuckingNoSort.setShootContext(shoot.getX(), shoot.getY(), 77);
                    fuckingNoSort.startRapidOuttake();
                    setPathState(20);
                }
                break;

            case 20:
                if (!follower.isBusy() && !fuckingNoSort.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
        if (actionTimer != null) actionTimer.resetTimer();
    }

    @Override
    public void init() {
        intkM = new IntakeMotor(hardwareMap);
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");

        Shooter = new LauncherSubsystem(flywheelMotorOne, flywheelMotorTwo, voltageSensor);

        Servo blocker = hardwareMap.get(Servo.class, "Blocker");
        Shooter.setBlocker(blocker);
        Shooter.init();

        indexerBase = new Indexer_Base(hardwareMap);
        fuckingNoSort = new Indexer_Rapid(hardwareMap, indexerBase, Shooter);

        buildPaths();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        double distance = getDistanceToGoal();

        fuckingNoSort.update();
        fuckingNoSort.setShootContext(shoot.getX(), shoot.getY(), distance);

        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("PathState", pathState);
        telemetry.addData("Rapid busy", fuckingNoSort.isBusy());
        telemetry.update();
    }

    public double getDistanceToGoal() {
        double gx = 0;
        double gy = 140;
        double dx = gx - follower.getPose().getX();
        double dy = gy - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }

    @Override
    public void stop() {
        if (fuckingNoSort != null) fuckingNoSort.stopAll();
        if (intkM != null) intkM.stop();
        if (indexerBase != null && indexerBase.intkM != null) indexerBase.intkM.stop();
    }
}
