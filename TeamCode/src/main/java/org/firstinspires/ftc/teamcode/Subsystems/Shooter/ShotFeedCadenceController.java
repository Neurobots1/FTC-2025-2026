package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;

public final class ShotFeedCadenceController {

    public enum State {
        IDLE,
        ARMED_WAITING_READY,
        GATE_OPENING,
        FEEDING
    }

    private final ElapsedTime gateOpenTimer = new ElapsedTime();
    private boolean armed;
    private boolean farZoneFeedPaused;
    private State state = State.IDLE;

    public void setArmed(boolean armed) {
        boolean wasArmed = this.armed;
        this.armed = armed;
        if (!armed) {
            farZoneFeedPaused = false;
            state = State.IDLE;
            gateOpenTimer.reset();
        } else if (!wasArmed) {
            state = State.ARMED_WAITING_READY;
        }
    }

    public void update(boolean gateOpen,
                       PrecisionShooterSubsystem.TelemetrySnapshot shooterSnapshot) {
        if (!armed) {
            farZoneFeedPaused = false;
            state = State.IDLE;
            return;
        }

        if (!gateOpen) {
            farZoneFeedPaused = false;
            if (state != State.ARMED_WAITING_READY) {
                state = State.ARMED_WAITING_READY;
            }
            return;
        }

        if (state == State.ARMED_WAITING_READY) {
            state = State.GATE_OPENING;
            gateOpenTimer.reset();
            return;
        }

        if (state == State.GATE_OPENING
                && gateOpenTimer.seconds() >= ShooterConstants.feedOpenSettlingSeconds) {
            state = State.FEEDING;
        }

        updateFarZonePause(shooterSnapshot);
    }

    public boolean isArmed() {
        return armed;
    }

    public State getState() {
        return state;
    }

    public boolean isFarZoneFeedPaused() {
        return farZoneFeedPaused;
    }

    public boolean isBlockerSettledOpen() {
        return state == State.FEEDING;
    }

    public boolean shouldPreFeedIntake() {
        return armed && (state == State.ARMED_WAITING_READY || state == State.GATE_OPENING);
    }

    public boolean shouldForceFeedIntake() {
        return state == State.FEEDING && !farZoneFeedPaused;
    }

    private void updateFarZonePause(PrecisionShooterSubsystem.TelemetrySnapshot shooterSnapshot) {
        if (!isBlockerSettledOpen() || !shouldManageFarZoneFeed(shooterSnapshot)) {
            farZoneFeedPaused = false;
            return;
        }

        double rpmDrop = Math.max(0.0, shooterSnapshot.targetRpm - shooterSnapshot.actualRpm);
        double pauseThreshold = Math.max(
                ShooterConstants.farZoneFeedPauseDropRpm,
                shooterSnapshot.targetRpm * ShooterConstants.farZoneFeedPauseDropFraction
        );
        double resumeThreshold = Math.max(
                ShooterConstants.farZoneFeedResumeDropRpm,
                shooterSnapshot.targetRpm * ShooterConstants.farZoneFeedResumeDropFraction
        );

        if (farZoneFeedPaused) {
            farZoneFeedPaused = rpmDrop > resumeThreshold;
        } else {
            farZoneFeedPaused = rpmDrop >= pauseThreshold;
        }
    }

    private boolean shouldManageFarZoneFeed(PrecisionShooterSubsystem.TelemetrySnapshot shooterSnapshot) {
        return shooterSnapshot != null
                && shooterSnapshot.inShootingZone
                && shooterSnapshot.targetRpm > 1.0
                && shooterSnapshot.tableDistanceInches >= ShooterConstants.farZoneFeedPauseDistanceInches;
    }
}
