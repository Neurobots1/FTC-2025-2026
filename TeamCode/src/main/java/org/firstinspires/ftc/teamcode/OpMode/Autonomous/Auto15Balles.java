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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_NoSort;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.NoSortOutake;


@Autonomous(name = "Auto15Balles", group = "Examples")
public class Auto15Balles extends OpMode {

    private boolean wantShoot = false;

    private double shootX = 0;
    private double shootY = 0;
    private double shootDistance = 0;

    private ElapsedTime fukingOutakeTimer;
    private enum ActionState {IDLE, WAIT_FOR_START, START,  FINISH, RESET , WAIT}
    public ActionState Outtake = ActionState.IDLE;
    private Follower follower;
    private NoSortOutake fuckingNoSort;
    private IntakeMotor intkM;
    private DcMotorEx flywheelMotorOne, flywheelMotorTwo;
    private VoltageSensor voltageSensor;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private LauncherSubsystem Shooter;

    public final Pose startPose = new Pose(34, 134, Math.toRadians(91));
    private final Pose shoot = new Pose(47, 92, Math.toRadians(130));
    private final Pose intkStart1 = new Pose(42, 88, Math.toRadians(190));
    private final Pose intkFinal1 = new Pose(15, 82, Math.toRadians(190));
    private final Pose intkStart2 = new Pose(42, 62, Math.toRadians(180));
    private final Pose intkFinal2 = new Pose(7, 62, Math.toRadians(180));
    private final Pose intk2Control = new Pose(30, 65);
    private final Pose gateIntk = new Pose(13, 63, Math.toRadians(155));
    private final Pose gateIntkControl = new Pose(27, 50);
    private final Pose finalShootPose = new Pose(55, 105, Math.toRadians(140));

    public PathChain Shoot1, IntkSt1, IntkFi1, Shoot2, IntkSt2, IntkFi2, Shoot3, GateIntk1, Shoot4, GateIntk2, FiShoot;

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
                .addPath(new BezierCurve(gateIntk, gateIntkControl, shoot))
                .setLinearHeadingInterpolation(gateIntk.getHeading(), shoot.getHeading())
                .build();

        GateIntk2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, gateIntkControl, gateIntk))
                .setLinearHeadingInterpolation(shoot.getHeading(), gateIntk.getHeading())
                .build();

        FiShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntk, gateIntkControl, shoot))
                .setLinearHeadingInterpolation(gateIntk.getHeading(), shoot.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Shoot1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()|| actionTimer.getElapsedTimeSeconds() > 2) {
                   startNoSort();
                    setPathState(2);
                }
                break;

            case 2:
                if (actionTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(IntkSt1);
                    intkM.intake();
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() || actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(IntkFi1);
                    intkM.stop();
                    setPathState(4);
                }
                break;

            case 4:
                follower.followPath(Shoot2);
                setPathState(5);
                break;

            case 5:
                if (!follower.isBusy()|| actionTimer.getElapsedTimeSeconds() > 2) {
                    startNoSort();
                    setPathState(6);
                }
                break;

            case 6:
                follower.followPath(IntkSt2);
                intkM.intake();
                actionTimer.resetTimer();
                setPathState(7);
                break;

            case 7:
                if (!follower.isBusy() || actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(IntkFi2);
                    intkM.stop();
                    setPathState(8);
                }
                break;

            case 8:
                follower.followPath(Shoot3);
                setPathState(9);
                break;

            case 9:
                if (!follower.isBusy()|| actionTimer.getElapsedTimeSeconds() > 2) {
                   startNoSort();
                    setPathState(10);
                }
                break;

            case 10:
                follower.followPath(GateIntk1);
                intkM.intake();
                actionTimer.resetTimer();
                setPathState(101);
                break;

            case 101:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    intkM.stop();
                    setPathState(11);
                }
                break;

            case 11:
                follower.followPath(Shoot4);
                setPathState(12);
                break;

            case 12:
                if (!follower.isBusy()|| actionTimer.getElapsedTimeSeconds() > 2) {
                    startNoSort();
                    setPathState(13);
                }
                break;

            case 13:
                follower.followPath(GateIntk2);
                intkM.intake();
                actionTimer.resetTimer();
                setPathState(131);
                break;

            case 131:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    intkM.stop();
                    setPathState(14);
                }
                break;

            case 14:
                follower.followPath(FiShoot);
                setPathState(15);
                break;

            case 15:
                if (!follower.isBusy()|| actionTimer.getElapsedTimeSeconds() > 2) {
                   startNoSort();
                    setPathState(-1);
                }
                break;
        }
    }


    public boolean isBusy() {
        return (Outtake != ActionState.IDLE);
    }

    public void setShootContext(double x, double y, double distance) {
        this.shootX = x;
        this.shootY = y;
        this.shootDistance = distance;
    }
    public void startNoSort() {
        if (Outtake!= ActionState.IDLE) return;
        fukingOutakeTimer.reset();

        Outtake = ActionState.START;
    }
    public void LineOuttake() {
        switch (Outtake) {
            case IDLE:
                break;

            case START:
                wantShoot = true;
                if (Shooter.flywheelReady()) {
                    intkM.slowOuttake();

                    fukingOutakeTimer.reset();
                    Outtake = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (fukingOutakeTimer.seconds()>2) {
                    wantShoot = false;
                    intkM.stop();
                }
                break;
        }

    }

    public void stopAll () {
        Outtake = ActionState.IDLE;
        wantShoot = false;
        if (intkM != null) intkM.stop();
        if (Shooter != null) Shooter.setFlywheelTicks(0);
    }


    public void update() {
        Shooter.updateShootingAuto(wantShoot, shootX, shootY, shootDistance);
        Shooter.update();



        if (!isBusy() && !wantShoot) {
            Shooter.setFlywheelTicks(0);
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
        if (actionTimer != null) actionTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        fuckingNoSort = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");


        intkM = new IntakeMotor(hardwareMap);

        Shooter = new LauncherSubsystem(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        fuckingNoSort = new NoSortOutake(hardwareMap, Shooter);

        Servo blocker = hardwareMap.get(Servo.class, "Blocker");
        Shooter.setBlocker(blocker);
        Shooter.init();

        buildPaths();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("PathState", pathState);

        telemetry.update();
    }

    @Override
    public void stop() {
        if (intkM != null) intkM.stop();
    }
}
