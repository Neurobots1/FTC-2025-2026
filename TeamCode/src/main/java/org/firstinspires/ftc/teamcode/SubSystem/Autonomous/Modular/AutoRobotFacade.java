package org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction;
import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.SortPattern;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.AutoShooterController;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;

public class AutoRobotFacade {

    private final Follower follower;
    private final SortedAutoController sortedController;
    private final AutoShooterController unsortedShooter;
    private final GoalSupplier goalSupplier;

    public interface GoalSupplier {
        double goalX();
        double goalY();
    }

    public AutoRobotFacade(Follower follower,
                           SortedAutoController sortedController,
                           AutoShooterController unsortedShooter,
                           GoalSupplier goalSupplier) {
        this.follower = follower;
        this.sortedController = sortedController;
        this.unsortedShooter = unsortedShooter;
        this.goalSupplier = goalSupplier;
    }

    public Follower follower() {
        return follower;
    }

    public AutoAction waitForFollowerIdle() {
        return Actions.waitUntil(() -> follower == null || !follower.isBusy());
    }

    public void updateSystems() {
        Pose pose = follower.getPose();
        double distance = Math.hypot(goalSupplier.goalX() - pose.getX(), goalSupplier.goalY() - pose.getY());

        if (sortedController != null) {
            sortedController.setShootContext(pose.getX(), pose.getY(), distance);
            sortedController.update();
        }

        if (unsortedShooter != null) {
            unsortedShooter.setGoalPosition(goalSupplier.goalX(), goalSupplier.goalY());
            unsortedShooter.update();
        }
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

    public AutoAction lockSortPattern(AprilTagPipeline aprilTag, double timeoutSeconds, SortPattern fallbackPattern) {
        if (sortedController == null) {
            return Actions.instant(() -> {});
        }
        return sortedController.lockPatternFromAprilTag(aprilTag, timeoutSeconds, fallbackPattern);
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
