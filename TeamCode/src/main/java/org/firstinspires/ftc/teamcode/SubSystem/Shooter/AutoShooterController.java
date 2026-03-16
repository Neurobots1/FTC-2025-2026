package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

public class AutoShooterController {

    public enum FeedMode {
        AUTO,
        DRIVER
    }

    private final LauncherSubsystem launcher;
    private final IntakeMotor intake;
    private final ElapsedTime feedTimer = new ElapsedTime();

    private FeedMode feedMode = FeedMode.AUTO;
    private boolean enabled;
    private boolean autoFeedRequested;
    private boolean driverGateRequested;
    private boolean feeding;
    private double aimX;
    private double aimY;
    private double aimDistance;

    public static double AUTO_FEED_SECONDS = 0.45;

    public AutoShooterController(LauncherSubsystem launcher, IntakeMotor intake) {
        this.launcher = launcher;
        this.intake = intake;
    }

    public void setFeedMode(FeedMode feedMode) {
        this.feedMode = feedMode;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            feeding = false;
            autoFeedRequested = false;
            driverGateRequested = false;
            launcher.setBlockerOpen(false);
            intake.stop();
            launcher.setFlywheelTicks(0);
        }
    }

    public void setAimContext(double x, double y, double distance) {
        aimX = x;
        aimY = y;
        aimDistance = distance;
    }

    public void requestAutoFeed(boolean requested) {
        autoFeedRequested = requested;
        if (!requested) {
            feeding = false;
            launcher.setBlockerOpen(false);
            intake.stop();
        }
    }

    public void requestDriverGate(boolean requested) {
        driverGateRequested = requested;
    }

    public boolean isReadyToFeed() {
        return enabled && isInShootingZone() && launcher.isAtSpeed();
    }

    public boolean isInShootingZone() {
        return LauncherSubsystem.isInShootingZone(aimX, aimY);
    }

    public boolean isBusy() {
        return feeding || autoFeedRequested;
    }

    public void update() {
        if (!enabled) {
            launcher.setBlockerOpen(false);
            intake.stop();
            launcher.update();
            return;
        }

        launcher.aimAtTarget(aimX, aimY, aimDistance);
        launcher.update();

        if (feedMode == FeedMode.AUTO) {
            updateAutoFeed();
        } else {
            updateDriverFeed();
        }
    }

    private void updateAutoFeed() {
        if (!autoFeedRequested) {
            launcher.setBlockerOpen(false);
            intake.stop();
            feeding = false;
            return;
        }

        if (!feeding) {
            if (launcher.isAtSpeed()) {
                feeding = true;
                feedTimer.reset();
                launcher.setBlockerOpen(true);
                intake.slowIntake();
            } else {
                launcher.setBlockerOpen(false);
                intake.stop();
            }
            return;
        }

        launcher.setBlockerOpen(true);
        intake.slowIntake();
        if (feedTimer.seconds() >= AUTO_FEED_SECONDS) {
            autoFeedRequested = false;
            feeding = false;
            launcher.setBlockerOpen(false);
            intake.stop();
        }
    }

    private void updateDriverFeed() {
        if (driverGateRequested && launcher.isAtSpeed()) {
            launcher.setBlockerOpen(true);
        } else {
            launcher.setBlockerOpen(false);
        }
    }
}
