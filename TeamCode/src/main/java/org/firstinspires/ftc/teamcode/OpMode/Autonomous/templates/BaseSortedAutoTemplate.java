package org.firstinspires.ftc.teamcode.OpMode.Autonomous.templates;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoRobotFacade;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.SortedAutoController;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.LimelightTagReader;

public abstract class BaseSortedAutoTemplate extends BaseAllianceAuto {

    private SortedAutoController sortedController;
    private LimelightTagReader tagReader;

    @Override
    protected final AutoRobotFacade createRobot(Follower follower,
                                                ShooterConstants shooterConfig,
                                                AutoAlliance alliance,
                                                double goalX,
                                                double goalY,
                                                double aimGoalX,
                                                double aimGoalY) {
        // Define the blue start pose in your subclass, and the alliance helper will
        // mirror it automatically for red through paths().pose(...).
        follower.setStartingPose(startPose());

        PrecisionShooterSubsystem shooter = PrecisionShooterSubsystem.create(hardwareMap, follower, shooterConfig);
        shooter.setAlliance(
                alliance == AutoAlliance.BLUE
                        ? PrecisionShooterSubsystem.Alliance.BLUE
                        : PrecisionShooterSubsystem.Alliance.RED
        );
        shooter.setGoalPosition(goalX, goalY);
        shooter.setAimPosition(aimGoalX, aimGoalY);
        shooter.setAutoAimEnabled(true);
        shooter.setUseAutoShootingZoneRadius(true);

        sortedController = new SortedAutoController(hardwareMap, shooter);
        tagReader = new LimelightTagReader(hardwareMap);
        tagReader.start();

        return new AutoRobotFacade(
                follower,
                sortedController,
                null,
                shooter,
                new AutoRobotFacade.GoalSupplier() {
                    @Override
                    public double goalX() {
                        return goalX;
                    }

                    @Override
                    public double goalY() {
                        return goalY;
                    }

                    @Override
                    public double aimGoalX() {
                        return aimGoalX;
                    }

                    @Override
                    public double aimGoalY() {
                        return aimGoalY;
                    }
                }
        );
    }

    @Override
    protected void onAutoStop() {
        if (tagReader != null) {
            tagReader.stop();
        }
        if (sortedController != null) {
            sortedController.stopAll();
        }
    }

    @Override
    protected void addTelemetry() {
        telemetry.addData("Pattern", sortedController.getPattern());
    }

    // Return the already-mirrored starting pose that should be given to the follower.
    // In most autos this is `paths().pose(blueStartPose)`.
    protected abstract com.pedropathing.geometry.Pose startPose();

    protected final SortedAutoController sortedController() {
        return sortedController;
    }

    protected final LimelightTagReader tagReader() {
        return tagReader;
    }
}
