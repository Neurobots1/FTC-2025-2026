package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.ActionScheduler;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular.MirroredPathFactory;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular.AutoRobotFacade;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular.ModularAutoBuilder;
import org.firstinspires.ftc.teamcode.SubSystem.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.AutoShooterController;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Mock 15 Ball Auto", group = "Experimental")
public class Mock15BallAuto extends OpMode {

    public static AutoAlliance ALLIANCE = AutoAlliance.BLUE;
    public static double GATE_INTAKE_SECONDS = 1.5;
    public static double LINE_INTAKE_SPEED = 1.0;

    private final ActionScheduler scheduler = new ActionScheduler();

    private Follower follower;
    private IntakeMotor intake;
    private AutoShooterController shooterController;
    private AutoRobotFacade robot;
    private MirroredPathFactory paths;

    private PathChain toPreloadShot;
    private PathChain toLine2Start;
    private PathChain toLine2Finish;
    private PathChain line2ToShot;
    private PathChain shotToGate;
    private PathChain gateToShot;
    private PathChain shotToGateAgain;
    private PathChain gateToFinalShot;
    private PathChain shotToLine1Start;
    private PathChain line1StartToFinish;
    private PathChain line1ToFinalShot;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        paths = new MirroredPathFactory(follower, ALLIANCE);
        follower.setStartingPose(paths.pose(new Pose(20, 119, Math.toRadians(139))));

        intake = new IntakeMotor(hardwareMap);

        DcMotorEx flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        DcMotorEx flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        LauncherSubsystem launcher = new LauncherSubsystem(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        Servo blocker = hardwareMap.get(Servo.class, "Blocker");
        launcher.setBlocker(blocker);
        launcher.init();

        shooterController = new AutoShooterController(launcher, intake);
        shooterController.setFeedMode(AutoShooterController.FeedMode.AUTO);

        robot = new AutoRobotFacade(
                follower,
                null,
                shooterController,
                new AutoRobotFacade.GoalSupplier() {
                    @Override
                    public double goalX() {
                        return ALLIANCE == AutoAlliance.BLUE ? 0.0 : 140.0;
                    }

                    @Override
                    public double goalY() {
                        return 140.0;
                    }
                }
        );

        buildPaths();
        buildRoutine();
    }

    @Override
    public void loop() {
        follower.update();
        AutoPoseHandoff.savePose(hardwareMap.appContext, follower.getPose());
        robot.updateSystems();
        scheduler.update();

        telemetry.addData("Alliance", ALLIANCE);
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("RoutineIdle", scheduler.isIdle());
        telemetry.addData("ShooterReady", shooterController.isReadyToFeed());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            AutoPoseHandoff.savePose(hardwareMap.appContext, follower.getPose());
        }
        shooterController.setEnabled(false);
        intake.stop();
    }

    private void buildRoutine() {
        ModularAutoBuilder builder = new ModularAutoBuilder(robot);

        builder
                .shootUnsorted(toPreloadShot)
                .followAsync(toLine2Start, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(toLine2Finish, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .doAction(Actions.waitSeconds(0.9))
                .doAction(stopIntake())
                .shootUnsorted(line2ToShot)
                .followAsync(shotToGate, 1.0, true)
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .doAction(Actions.waitSeconds(GATE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .shootUnsorted(gateToShot)
                .followAsync(shotToGateAgain, 1.0, true)
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .doAction(Actions.waitSeconds(GATE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .shootUnsorted(gateToFinalShot)
                .followAsync(shotToLine1Start, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(line1StartToFinish, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .doAction(Actions.waitSeconds(0.9))
                .doAction(stopIntake())
                .shootUnsorted(line1ToFinalShot)
                .stopUnsortedShooter();

        scheduler.setAction(builder.build());
    }

    private org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction startTimedIntake() {
        return Actions.instant(new Runnable() {
            @Override
            public void run() {
                intake.intake();
            }
        });
    }

    private org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction stopIntake() {
        return Actions.instant(new Runnable() {
            @Override
            public void run() {
                intake.stop();
            }
        });
    }

    private void buildPaths() {
        Pose startPose = new Pose(20, 119, Math.toRadians(139));
        Pose preloadShot = new Pose(47, 92, Math.toRadians(139));
        Pose line2Start = new Pose(50, 55, Math.toRadians(180));
        Pose line2Finish = new Pose(13, 55, Math.toRadians(180));
        Pose line2Control = new Pose(45, 55);
        Pose gatePose = new Pose(11, 53, Math.toRadians(147));
        Pose gateControl = new Pose(27, 50);
        Pose line1Start = new Pose(50, 88, Math.toRadians(190));
        Pose line1Finish = new Pose(23, 82, Math.toRadians(190));
        Pose finalShot = new Pose(55, 105, Math.toRadians(145));

        toPreloadShot = paths.line(startPose, preloadShot);
        toLine2Start = paths.line(preloadShot, line2Start);
        toLine2Finish = paths.line(line2Start, line2Finish);
        line2ToShot = paths.curve(line2Finish, line2Control, preloadShot);
        shotToGate = paths.curve(preloadShot, gateControl, gatePose);
        gateToShot = paths.curve(gatePose, line2Control, preloadShot);
        shotToGateAgain = paths.curve(preloadShot, gateControl, gatePose);
        gateToFinalShot = paths.curve(gatePose, line2Control, finalShot);
        shotToLine1Start = paths.line(finalShot, line1Start);
        line1StartToFinish = paths.line(line1Start, line1Finish);
        line1ToFinalShot = paths.line(line1Finish, finalShot);
    }
}
