package org.firstinspires.ftc.teamcode.OpMode.Autonomous.templates;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoRobotFacade;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.AutoShooterController;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;

public abstract class BaseUnsortedAutoTemplate extends BaseAllianceAuto {

    private IntakeMotor intake;
    private AutoShooterController shooterController;

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

        intake = new IntakeMotor(hardwareMap);

        PrecisionShooterSubsystem shooter = PrecisionShooterSubsystem.create(hardwareMap, follower, shooterConfig);
        shooter.setAlliance(
                alliance == AutoAlliance.BLUE
                        ? PrecisionShooterSubsystem.Alliance.BLUE
                        : PrecisionShooterSubsystem.Alliance.RED
        );
        shooter.setGoalPosition(goalX, goalY);
        shooter.setAimPosition(aimGoalX, aimGoalY);
        shooter.setAutoAimEnabled(true);

        shooterController = new AutoShooterController(shooter, intake);
        shooterController.setFeedMode(AutoShooterController.FeedMode.AUTO);

        return new AutoRobotFacade(
                follower,
                null,
                shooterController,
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
        if (shooterController != null) {
            shooterController.setEnabled(false);
        }
        if (intake != null) {
            intake.stop();
        }
    }

    @Override
    protected void addTelemetry() {
        telemetry.addData("ShooterReady", shooterController.isReadyToFeed());
    }

    // Return the already-mirrored starting pose that should be given to the follower.
    // In most autos this is `paths().pose(blueStartPose)`.
    protected abstract com.pedropathing.geometry.Pose startPose();

    protected final IntakeMotor intake() {
        return intake;
    }

    protected final AutoShooterController shooterController() {
        return shooterController;
    }
}
