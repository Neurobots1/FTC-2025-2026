package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;

public class AutoShooterController {

    public enum FeedMode {
        AUTO,
        DRIVER
    }

    private final PrecisionShooterSubsystem shooter;
    private final IntakeMotor intake;
    private final ShotFeedCadenceController shotFeedController = new ShotFeedCadenceController();
    private final ElapsedTime autoFeedTimer = new ElapsedTime();

    private FeedMode feedMode = FeedMode.AUTO;
    private boolean enabled;
    private boolean autoFeedRequested;
    private boolean autoFeedTimingStarted;
    private boolean driverGateRequested;
    private double goalX;
    private double goalY;
    private double aimGoalX;
    private double aimGoalY;

    public AutoShooterController(PrecisionShooterSubsystem shooter, IntakeMotor intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    public void setFeedMode(FeedMode feedMode) {
        this.feedMode = feedMode;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        shooter.setSpinEnabled(enabled);
        shooter.setAutoAimEnabled(true);
        if (!enabled) {
            autoFeedRequested = false;
            autoFeedTimingStarted = false;
            driverGateRequested = false;
            shotFeedController.setArmed(false);
            shooter.requestFire(false);
            intake.stop();
        }
    }

    public void setGoalPosition(double x, double y) {
        goalX = x;
        goalY = y;
    }

    public void setAimPosition(double x, double y) {
        aimGoalX = x;
        aimGoalY = y;
    }

    public void requestAutoFeed(boolean requested) {
        if (requested && !autoFeedRequested) {
            autoFeedTimingStarted = false;
            autoFeedTimer.reset();
        }
        autoFeedRequested = requested;
        shotFeedController.setArmed(requested);
        if (!requested) {
            autoFeedTimingStarted = false;
            shooter.requestFire(false);
        }
    }

    public void requestDriverGate(boolean requested) {
        driverGateRequested = requested;
    }

    public boolean isReadyToFeed() {
        return enabled && shooter.isInShootingZone() && shooter.isReadyToShootNow();
    }

    public boolean isInShootingZone() {
        return shooter.isInShootingZone();
    }

    public boolean isBusy() {
        return autoFeedRequested || shotFeedController.isArmed() || shooter.isFeedGateOpen();
    }

    public boolean wantsGoalTrackingControl() {
        return enabled
                && shooter.shouldUseChassisHeadingLock()
                && shooter.isInShootingZone();
    }

    public void update() {
        shooter.setGoalPosition(goalX, goalY);
        shooter.setAimPosition(aimGoalX, aimGoalY);
        shooter.setSpinEnabled(enabled);
        shooter.setAutoAimEnabled(true);

        if (!enabled) {
            shotFeedController.setArmed(false);
            shooter.requestFire(false);
            intake.stop();
            shooter.update();
            return;
        }

        if (feedMode == FeedMode.AUTO) {
            shotFeedController.setArmed(autoFeedRequested);
            shooter.requestFire(autoFeedRequested && shooter.isInShootingZone());
        } else {
            shotFeedController.setArmed(driverGateRequested);
            shooter.requestFire(driverGateRequested);
        }

        shooter.update();
        PrecisionShooterSubsystem.TelemetrySnapshot shooterSnapshot = shooter.snapshot();
        shotFeedController.update(shooter.isFeedGateOpen(), shooterSnapshot);

        if (feedMode == FeedMode.AUTO) {
            updateAutoFeed();
        } else {
            updateDriverFeed();
        }
    }

    private void updateAutoFeed() {
        if (!autoFeedRequested) {
            shotFeedController.setArmed(false);
            autoFeedTimingStarted = false;
            return;
        }

        if (shotFeedController.shouldForceFeedIntake()) {
            intake.slowIntake();
        } else if (shotFeedController.shouldPreFeedIntake()) {
            intake.preFeedIntake();
        } else {
            intake.stop();
        }

        if (shooter.isFeedGateOpen() && !autoFeedTimingStarted) {
            autoFeedTimer.reset();
            autoFeedTimingStarted = true;
        }

        if (autoFeedTimingStarted
                && autoFeedTimer.seconds() >= ShooterConstants.autoShotFeedDurationSeconds) {
            autoFeedRequested = false;
            autoFeedTimingStarted = false;
            shotFeedController.setArmed(false);
            shooter.requestFire(false);
            intake.stop();
        }
    }

    private void updateDriverFeed() {
        intake.stop();
    }
}
