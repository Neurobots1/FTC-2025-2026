package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;

public class AutoShooterController {

    public enum FeedMode {
        AUTO,
        DRIVER
    }

    private final PrecisionShooterSubsystem shooter;
    private final IntakeMotor intake;
    private final ElapsedTime feedTimer = new ElapsedTime();

    private FeedMode feedMode = FeedMode.AUTO;
    private boolean enabled;
    private boolean autoFeedRequested;
    private boolean driverGateRequested;
    private boolean feeding;
    private double goalX;
    private double goalY;

    public static double AUTO_FEED_SECONDS = 0.45;

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
        shooter.setAutoAimEnabled(enabled);
        if (!enabled) {
            feeding = false;
            autoFeedRequested = false;
            driverGateRequested = false;
            shooter.requestFire(false);
            intake.stop();
        }
    }

    public void setGoalPosition(double x, double y) {
        goalX = x;
        goalY = y;
    }

    public void requestAutoFeed(boolean requested) {
        autoFeedRequested = requested;
        if (!requested) {
            feeding = false;
            shooter.requestFire(false);
            intake.stop();
        }
    }

    public void requestDriverGate(boolean requested) {
        driverGateRequested = requested;
    }

    public boolean isReadyToFeed() {
        return enabled && shooter.isReadyToShootNow();
    }

    public boolean isInShootingZone() {
        return shooter.isInShootingZone();
    }

    public boolean isBusy() {
        return feeding || autoFeedRequested;
    }

    public void update() {
        shooter.setGoalPosition(goalX, goalY);
        shooter.setSpinEnabled(enabled);
        shooter.setAutoAimEnabled(enabled);

        if (!enabled) {
            shooter.requestFire(false);
            intake.stop();
            shooter.update();
            return;
        }

        if (feedMode == FeedMode.AUTO) {
            updateAutoFeed();
        } else {
            updateDriverFeed();
        }

        shooter.update();
    }

    private void updateAutoFeed() {
        if (!autoFeedRequested) {
            shooter.requestFire(false);
            intake.stop();
            feeding = false;
            return;
        }

        shooter.requestFire(true);

        if (!feeding) {
            if (shooter.isReadyToShootNow()) {
                feeding = true;
                feedTimer.reset();
                intake.slowIntake();
            } else {
                intake.stop();
            }
            return;
        }

        intake.slowIntake();
        if (feedTimer.seconds() >= AUTO_FEED_SECONDS) {
            autoFeedRequested = false;
            feeding = false;
            shooter.requestFire(false);
            intake.stop();
        }
    }

    private void updateDriverFeed() {
        shooter.requestFire(driverGateRequested);
    }
}
