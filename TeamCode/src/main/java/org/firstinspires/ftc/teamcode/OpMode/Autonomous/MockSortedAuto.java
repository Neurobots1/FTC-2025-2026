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
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular.AutoRobotFacade;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular.MirroredPathFactory;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular.ModularAutoBuilder;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular.SortedAutoController;
import org.firstinspires.ftc.teamcode.SubSystem.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.SortPattern;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Mock Sorted Auto", group = "Experimental")
public class MockSortedAuto extends OpMode {

    public static AutoAlliance ALLIANCE = AutoAlliance.BLUE;

    private final ActionScheduler scheduler = new ActionScheduler();

    private Follower follower;
    private SortedAutoController sortedController;
    private AutoRobotFacade robot;
    private MirroredPathFactory paths;
    private AprilTagPipeline aprilTag;

    private PathChain toPreloadShot;
    private PathChain toLine1Start;
    private PathChain toLine1Finish;
    private PathChain line1ToShot;
    private PathChain toLine2Start;
    private PathChain toLine2Finish;
    private PathChain line2ToFinalShot;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        paths = new MirroredPathFactory(follower, ALLIANCE);
        follower.setStartingPose(paths.pose(new Pose(20, 119, Math.toRadians(139))));

        DcMotorEx flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        DcMotorEx flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        LauncherSubsystem launcher = new LauncherSubsystem(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        Servo blocker = hardwareMap.get(Servo.class, "Blocker");
        launcher.setBlocker(blocker);
        launcher.init();

        sortedController = new SortedAutoController(hardwareMap, launcher);
        aprilTag = new AprilTagPipeline(hardwareMap);
        aprilTag.startCamera();

        robot = new AutoRobotFacade(
                follower,
                sortedController,
                null,
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
        telemetry.addData("Pattern", sortedController.getPattern());
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("RoutineIdle", scheduler.isIdle());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            AutoPoseHandoff.savePose(hardwareMap.appContext, follower.getPose());
        }
        sortedController.stopAll();
    }

    private void buildRoutine() {
        ModularAutoBuilder builder = new ModularAutoBuilder(robot);
        builder
                .readSortPattern(aprilTag, 1.0, SortPattern.NOSORT)
                .shootSortedPreload(toPreloadShot)
                .followAsync(toLine1Start, 1.0, true)
                .waitForFollowerIdle()
                .followAsync(toLine1Finish, 1.0, true)
                .waitForFollowerIdle()
                .doAction(robot.enableSortedPreSpin(true))
                .doAction(robot.startSortedIntake(1))
                .doAction(robot.waitForSortedIdle())
                .followAsync(line1ToShot, 1.0, true)
                .waitForFollowerIdle()
                .doAction(robot.startSortedShot(1))
                .doAction(robot.waitForSortedIdle())
                .followAsync(toLine2Start, 1.0, true)
                .waitForFollowerIdle()
                .followAsync(toLine2Finish, 1.0, true)
                .waitForFollowerIdle()
                .doAction(robot.enableSortedPreSpin(true))
                .doAction(robot.startSortedIntake(2))
                .doAction(robot.waitForSortedIdle())
                .followAsync(line2ToFinalShot, 1.0, true)
                .waitForFollowerIdle()
                .doAction(robot.startSortedShot(2))
                .doAction(robot.waitForSortedIdle())
                .stopSortedShooter();

        scheduler.setAction(builder.build());
    }

    private void buildPaths() {
        Pose startPose = new Pose(20, 119, Math.toRadians(139));
        Pose preloadShot = new Pose(47, 92, Math.toRadians(139));
        Pose line1Start = new Pose(50, 88, Math.toRadians(190));
        Pose line1Finish = new Pose(23, 82, Math.toRadians(190));
        Pose line2Start = new Pose(50, 55, Math.toRadians(180));
        Pose line2Finish = new Pose(15, 55, Math.toRadians(180));
        Pose line2Control = new Pose(45, 55);
        Pose finalShot = new Pose(55, 105, Math.toRadians(145));

        toPreloadShot = paths.line(startPose, preloadShot);
        toLine1Start = paths.line(preloadShot, line1Start);
        toLine1Finish = paths.line(line1Start, line1Finish);
        line1ToShot = paths.line(line1Finish, preloadShot);
        toLine2Start = paths.line(preloadShot, line2Start);
        toLine2Finish = paths.line(line2Start, line2Finish);
        line2ToFinalShot = paths.curve(line2Finish, line2Control, finalShot);
    }
}
