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

@Autonomous(name = "Auto15Balles_Red", group = "Examples")
public class Auto15Balles_Red extends OpMode {

    private Follower follower;

    private Indexer_Base indexerBase;
    private Indexer_Rapid rapidIndexer;
    private IntakeMotor intkM;

    private DcMotorEx flywheelMotorOne, flywheelMotorTwo;
    private VoltageSensor voltageSensor;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private LauncherSubsystem Shooter;

    public final Pose startPose = new Pose(124, 119, Math.toRadians(41));
    private final Pose Shoot = new Pose(97, 92, Math.toRadians(41));
    private final Pose IntkStart1 = new Pose(94, 88, Math.toRadians(350));
    private final Pose IntkFinal1 = new Pose(121, 82, Math.toRadians(350));
    private final Pose IntkStart2 = new Pose(94, 55, Math.toRadians(0));
    private final Pose IntkFinal2 = new Pose(131, 55, Math.toRadians(0));
    private final Pose ControlIntk2 = new Pose(99, 55);
    private final Pose gateIntk = new Pose(133, 53, Math.toRadians(32));
    private final Pose gateIntkControl = new Pose(117, 50);
    private final Pose FinalShootPose = new Pose(89, 105, Math.toRadians(35));

    public PathChain Shoot1, IntkSt1, IntkFi1, Shoot2, IntkSt2, IntkFi2, Shoot3, GateIntk1, Shoot4, GateIntk2, FiShoot;

    private boolean gateWaitStarted1 = false;
    private boolean gateWaitStarted2 = false;

    public void buildPaths() {
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        IntkSt1 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart1))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart1.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        IntkFi1 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart1, IntkFinal1))
                .setLinearHeadingInterpolation(IntkStart1.getHeading(), IntkFinal1.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal1, FinalShootPose))
                .setLinearHeadingInterpolation(IntkFinal1.getHeading(), FinalShootPose.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        IntkSt2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart2))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart2.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        IntkFi2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart2, IntkFinal2))
                .setLinearHeadingInterpolation(IntkStart2.getHeading(), IntkFinal2.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(IntkFinal2, ControlIntk2, Shoot))
                .setLinearHeadingInterpolation(IntkFinal2.getHeading(), Shoot.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        GateIntk1 = follower.pathBuilder()
                .addPath(new BezierCurve(Shoot, gateIntkControl, gateIntk))
                .setLinearHeadingInterpolation(Shoot.getHeading(), gateIntk.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntk, ControlIntk2, Shoot))
                .setLinearHeadingInterpolation(gateIntk.getHeading(), Shoot.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        GateIntk2 = follower.pathBuilder()
                .addPath(new BezierCurve(Shoot, gateIntkControl, gateIntk))
                .setLinearHeadingInterpolation(Shoot.getHeading(), gateIntk.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        FiShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntk, ControlIntk2, Shoot))
                .setLinearHeadingInterpolation(gateIntk.getHeading(), Shoot.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                rapidIndexer.startSetupOuttake();
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot1, 1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && Shooter.flywheelReady()) {
                    rapidIndexer.startRapidOuttake();
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && !rapidIndexer.isBusy()) {
                    follower.followPath(IntkSt2, 1, false);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi2, 1, false);
                    rapidIndexer.startRapidIntakeNoSort();
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot3, 1, true);
                    intkM.stop();
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    rapidIndexer.setShootContext(Shoot.getX(), Shoot.getY(), 77);
                    rapidIndexer.startRapidOuttake();
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy() && !rapidIndexer.isBusy()) {
                    follower.followPath(GateIntk1, 1, true);
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
                    if (actionTimer.getElapsedTimeSeconds() > 1.75) {
                        intkM.stop();
                        follower.followPath(Shoot4, 1, true);
                        gateWaitStarted1 = false;
                        setPathState(11);
                    }
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    intkM.stop();
                    rapidIndexer.setShootContext(Shoot.getX(), Shoot.getY(), 77);
                    rapidIndexer.startRapidOuttake();
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy() && !rapidIndexer.isBusy()) {
                    follower.followPath(GateIntk2, 1, true);
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
                    if (actionTimer.getElapsedTimeSeconds() > 1.754) {
                        intkM.stop();
                        follower.followPath(FiShoot, 1, true);
                        gateWaitStarted2 = false;
                        setPathState(15);
                    }
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    intkM.stop();
                    rapidIndexer.setShootContext(Shoot.getX(), Shoot.getY(), 77);
                    rapidIndexer.startRapidOuttake();
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy() && !rapidIndexer.isBusy()) {
                    follower.followPath(IntkSt1, 1, false);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(IntkFi1, 1, false);
                    rapidIndexer.startRapidIntakeNoSort();
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, 1, true);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    intkM.stop();
                    rapidIndexer.setShootContext(Shoot.getX(), Shoot.getY(), 77);
                    rapidIndexer.startRapidOuttake();
                    setPathState(20);
                }
                break;

            case 20:
                if (!follower.isBusy() && !rapidIndexer.isBusy()) {
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
        rapidIndexer = new Indexer_Rapid(hardwareMap, indexerBase, Shooter);

        buildPaths();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        double distance = getDistanceToGoal();

        rapidIndexer.update();
        rapidIndexer.setShootContext(Shoot.getX(), Shoot.getY(), distance);

        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("PathState", pathState);
        telemetry.addData("Rapid busy", rapidIndexer.isBusy());
        telemetry.update();
    }

    public double getDistanceToGoal() {
        double gx = 140;
        double gy = 140;
        double dx = gx - follower.getPose().getX();
        double dy = gy - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }

    @Override
    public void stop() {
        if (rapidIndexer != null) rapidIndexer.stopAll();
        if (intkM != null) intkM.stop();
        if (indexerBase != null && indexerBase.intkM != null) indexerBase.intkM.stop();
    }
}