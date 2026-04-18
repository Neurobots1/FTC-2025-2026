package org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.AutoAction;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.SortPattern;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.AutoShooterController;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ChassisHeadingLockController;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.LimelightTagReader;

public class AutoRobotFacade {

    private final Follower follower;
    private final SortedAutoController sortedController;
    private final AutoShooterController unsortedShooter;
    private final PrecisionShooterSubsystem shooter;
    private final GoalSupplier goalSupplier;
    private final ChassisHeadingLockController headingLockController = new ChassisHeadingLockController();
    private boolean headingLockDriveActive;

    public interface GoalSupplier {
        double goalX();
        double goalY();
        double aimGoalX();
        double aimGoalY();
    }

    public AutoRobotFacade(Follower follower,
                           SortedAutoController sortedController,
                           AutoShooterController unsortedShooter,
                           PrecisionShooterSubsystem shooter,
                           GoalSupplier goalSupplier) {
        this.follower = follower;
        this.sortedController = sortedController;
        this.unsortedShooter = unsortedShooter;
        this.shooter = shooter;
        this.goalSupplier = goalSupplier;
    }

    public Follower follower() {
        return follower;
    }

    public AutoAction waitForFollowerIdle() {
        return Actions.waitUntil(() -> follower == null || !follower.isBusy());
    }

    public void updateDriveControl() {
        if (follower == null || shooter == null) {
            return;
        }

        if (follower.isBusy()) {
            headingLockController.reset();
            headingLockDriveActive = false;
            return;
        }

        if (shouldApplyHeadingLock()) {
            follower.startTeleopDrive();
            double turnCommand = headingLockController.update(shooter, shooter.snapshot());
            follower.setTeleOpDrive(0.0, 0.0, turnCommand, true);
            headingLockDriveActive = true;
            return;
        }

        headingLockController.reset();
        if (headingLockDriveActive) {
            follower.startTeleopDrive();
            follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
            headingLockDriveActive = false;
        }
    }

    public void updateSystems() {
        if (shooter != null) {
            shooter.setGoalPosition(goalSupplier.goalX(), goalSupplier.goalY());
            shooter.setAimPosition(goalSupplier.aimGoalX(), goalSupplier.aimGoalY());
        }

        Pose pose = follower.getPose();
        double distance = Math.hypot(goalSupplier.goalX() - pose.getX(), goalSupplier.goalY() - pose.getY());

        if (sortedController != null) {
            sortedController.setShootContext(pose.getX(), pose.getY(), distance);
            sortedController.update();
        }

        if (unsortedShooter != null) {
            unsortedShooter.setGoalPosition(goalSupplier.goalX(), goalSupplier.goalY());
            unsortedShooter.setAimPosition(goalSupplier.aimGoalX(), goalSupplier.aimGoalY());
            unsortedShooter.update();
        }
    }

    public void updateInitSystems() {
        if (shooter == null) {
            return;
        }

        shooter.setGoalPosition(goalSupplier.goalX(), goalSupplier.goalY());
        shooter.setAimPosition(goalSupplier.aimGoalX(), goalSupplier.aimGoalY());
        shooter.update();
    }

    public boolean isShooterTurretHomed() {
        return shooter == null || shooter.isTurretHomed();
    }

    public void saveTurretHome(android.content.Context context) {
        if (shooter != null) {
            shooter.saveTurretHome(context);
        }
    }

    private boolean shouldApplyHeadingLock() {
        if (!shooter.shouldUseChassisHeadingLock()) {
            return false;
        }
        if (sortedController != null && sortedController.wantsGoalTrackingControl()) {
            return true;
        }
        return unsortedShooter != null && unsortedShooter.wantsGoalTrackingControl();
    }

    public AutoAction enableSortedPreSpin(boolean enabled) {
        return Actions.instant(() -> {
            if (sortedController != null) {
                sortedController.setPreSpinEnabled(enabled);
            }
        });
    }

    public AutoAction stopSortedShooter() {
        return enableSortedPreSpin(false);
    }

    public AutoAction startSortedIntake(int line) {
        return Actions.instant(() -> {
            if (sortedController != null) {
                sortedController.startIntakeLine(line);
            }
        });
    }

    public AutoAction startSortedShot(int line) {
        return Actions.instant(() -> {
            if (sortedController != null) {
                sortedController.startShootLine(line);
            }
        });
    }

    public AutoAction startSortedPreloadShot() {
        return Actions.instant(() -> {
            if (sortedController != null) {
                sortedController.startPreloadShot();
            }
        });
    }

    public AutoAction lockSortPattern(LimelightTagReader tagReader, double timeoutSeconds, SortPattern fallbackPattern) {
        if (sortedController == null) {
            return Actions.instant(() -> {});
        }
        return sortedController.lockPatternFromAprilTag(tagReader, timeoutSeconds, fallbackPattern);
    }

    public AutoAction waitForSortedIdle() {
        return Actions.waitUntil(() -> sortedController == null || !sortedController.isBusy());
    }

    public AutoAction waitForSortedShotZone() {
        return Actions.waitUntil(() -> sortedController == null || sortedController.isInShootingZone());
    }

    public AutoAction waitForSortedReadyToShoot() {
        return Actions.waitUntil(() -> sortedController == null || sortedController.isReadyToShoot());
    }

    public AutoAction enableUnsortedShooter(boolean enabled) {
        return Actions.instant(() -> {
            if (unsortedShooter != null) {
                unsortedShooter.setEnabled(enabled);
            }
        });
    }

    public AutoAction startUnsortedShot() {
        return Actions.instant(() -> {
            if (unsortedShooter != null) {
                unsortedShooter.requestAutoFeed(true);
            }
        });
    }

    public AutoAction waitForUnsortedShotDone() {
        return Actions.waitUntil(() -> unsortedShooter == null || !unsortedShooter.isBusy());
    }

    public AutoAction waitForUnsortedShotZone() {
        return Actions.waitUntil(() -> unsortedShooter == null || unsortedShooter.isInShootingZone());
    }

    public AutoAction waitForUnsortedReadyToShoot() {
        return Actions.waitUntil(() -> unsortedShooter == null || unsortedShooter.isReadyToFeed());
    }
}
