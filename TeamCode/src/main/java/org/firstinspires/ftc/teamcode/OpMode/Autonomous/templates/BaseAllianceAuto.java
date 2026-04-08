package org.firstinspires.ftc.teamcode.OpMode.Autonomous.templates;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants.PedroConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TeleopConstants;
import org.firstinspires.ftc.teamcode.Subsystems.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.ActionScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoRobotFacade;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AllianceMirroring;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.MirroredPathFactory;

public abstract class BaseAllianceAuto extends OpMode {

    private final ActionScheduler scheduler = new ActionScheduler();

    private Follower follower;
    private AutoRobotFacade robot;
    private MirroredPathFactory paths;
    private ShooterConstants shooterConfig;
    private double goalX;
    private double goalY;
    private double aimGoalX;
    private double aimGoalY;

    protected abstract AutoAlliance alliance();

    // Create the robot-specific subsystems/controllers here.
    // This should return the facade used by the shared scheduler/update loop.
    protected abstract AutoRobotFacade createRobot(Follower follower,
                                                   ShooterConstants shooterConfig,
                                                   AutoAlliance alliance,
                                                   double goalX,
                                                   double goalY,
                                                   double aimGoalX,
                                                   double aimGoalY);

    // Build all Pedro paths here before buildRoutine() runs.
    protected abstract void buildPaths();

    // Build the action sequence here and finish with scheduler().setAction(...).
    protected abstract void buildRoutine();

    // Optional hook after robot creation and before paths/routine are built.
    protected void onPostRobotInit() {
    }

    // Optional hook for shutting down extra hardware/controllers.
    protected void onAutoStop() {
    }

    // Optional hook for auto-specific telemetry lines.
    protected void addTelemetry() {
    }

    @Override
    public void init() {
        follower = PedroConstants.createFollower(hardwareMap);
        shooterConfig = new ShooterConstants();

        AutoAlliance alliance = alliance();
        Pose goalPose = AllianceMirroring.forAlliance(TeleopConstants.BLUE_GOAL_POSE, alliance);
        Pose aimGoalPose = AllianceMirroring.forAlliance(TeleopConstants.BLUE_HEADING_AIM_POSE, alliance);
        goalX = goalPose.getX();
        goalY = goalPose.getY();
        aimGoalX = aimGoalPose.getX();
        aimGoalY = aimGoalPose.getY();

        paths = new MirroredPathFactory(
                follower,
                alliance,
                goalX,
                goalY,
                !shooterConfig.turretEnabled && shooterConfig.lockChassisHeadingWhenTurretDisabled
        );

        robot = createRobot(follower, shooterConfig, alliance, goalX, goalY, aimGoalX, aimGoalY);
        onPostRobotInit();
        buildPaths();
        buildRoutine();
    }

    @Override
    public void loop() {
        robot.updateDriveControl();
        follower.update();
        AutoPoseHandoff.savePose(hardwareMap.appContext, follower.getPose());
        robot.updateSystems();
        scheduler.update();

        telemetry.addData("Alliance", alliance());
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("RoutineIdle", scheduler.isIdle());
        addTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            AutoPoseHandoff.savePose(hardwareMap.appContext, follower.getPose());
        }
        onAutoStop();
    }

    protected final Follower follower() {
        return follower;
    }

    protected final AutoRobotFacade robot() {
        return robot;
    }

    protected final MirroredPathFactory paths() {
        return paths;
    }

    protected final ShooterConstants shooterConfig() {
        return shooterConfig;
    }

    protected final double goalX() {
        return goalX;
    }

    protected final double goalY() {
        return goalY;
    }

    protected final double aimGoalX() {
        return aimGoalX;
    }

    protected final double aimGoalY() {
        return aimGoalY;
    }

    protected final ActionScheduler scheduler() {
        return scheduler;
    }
}
