package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;

public final class AutoShotBurstTracker {
    private final ElapsedTime burstTimer = new ElapsedTime();
    private final ElapsedTime sampleTimer = new ElapsedTime();

    private enum DetectionState {
        WAITING_FOR_DROP,
        TRACKING_DIP,
        WAITING_FOR_RESET
    }

    private boolean active;
    private boolean filteredRpmInitialized;
    private boolean burstWindowStarted;
    private int expectedShots;
    private int detectedShots;
    private double filteredRpm;
    private double previousFilteredRpm;
    private double deepestDropRpm;
    private double lastDropRpm;
    private double lastSlopeRpmPerSecond;
    private DetectionState detectionState = DetectionState.WAITING_FOR_DROP;

    public void start(int expectedShots) {
        active = true;
        this.expectedShots = Math.max(1, expectedShots);
        detectedShots = 0;
        filteredRpmInitialized = false;
        burstWindowStarted = false;
        filteredRpm = 0.0;
        previousFilteredRpm = 0.0;
        deepestDropRpm = 0.0;
        lastDropRpm = 0.0;
        lastSlopeRpmPerSecond = 0.0;
        detectionState = DetectionState.WAITING_FOR_DROP;
        burstTimer.reset();
        sampleTimer.reset();
    }

    public void stop() {
        active = false;
        expectedShots = 0;
        detectedShots = 0;
        filteredRpmInitialized = false;
        burstWindowStarted = false;
        filteredRpm = 0.0;
        previousFilteredRpm = 0.0;
        deepestDropRpm = 0.0;
        lastDropRpm = 0.0;
        lastSlopeRpmPerSecond = 0.0;
        detectionState = DetectionState.WAITING_FOR_DROP;
    }

    public void update(PrecisionShooterSubsystem.TelemetrySnapshot shooterSnapshot,
                       boolean detectionEnabled) {
        if (!active || shooterSnapshot == null || shooterSnapshot.targetRpm <= 1.0) {
            return;
        }

        double dt = Math.max(1e-3, sampleTimer.seconds());
        sampleTimer.reset();

        if (!filteredRpmInitialized) {
            filteredRpm = shooterSnapshot.actualRpm;
            previousFilteredRpm = filteredRpm;
            filteredRpmInitialized = true;
            return;
        }

        previousFilteredRpm = filteredRpm;
        filteredRpm += (shooterSnapshot.actualRpm - filteredRpm)
                * ShooterConstants.autoShotDetectFilterGain;

        if (detectionEnabled && !burstWindowStarted) {
            burstWindowStarted = true;
            burstTimer.reset();
        }

        double rpmDrop = Math.max(0.0, shooterSnapshot.targetRpm - filteredRpm);
        double rpmSlope = (filteredRpm - previousFilteredRpm) / dt;
        lastDropRpm = rpmDrop;
        lastSlopeRpmPerSecond = rpmSlope;
        double shotThreshold = Math.max(
                ShooterConstants.autoShotDetectDropRpm,
                shooterSnapshot.targetRpm * ShooterConstants.autoShotDetectDropFraction
        );
        double resetThreshold = Math.max(
                ShooterConstants.autoShotDetectResetDropRpm,
                shooterSnapshot.targetRpm * ShooterConstants.autoShotDetectResetDropFraction
        );
        double recoveryRiseThreshold = Math.max(
                ShooterConstants.autoShotDetectRecoveryRiseRpm,
                shooterSnapshot.targetRpm * ShooterConstants.autoShotDetectRecoveryRiseFraction
        );

        switch (detectionState) {
            case WAITING_FOR_DROP:
                if (detectionEnabled
                        && rpmDrop >= shotThreshold
                        && rpmSlope <= -Math.abs(ShooterConstants.autoShotDetectFallSlopeRpmPerSecond)) {
                    detectionState = DetectionState.TRACKING_DIP;
                    deepestDropRpm = rpmDrop;
                }
                break;

            case TRACKING_DIP:
                deepestDropRpm = Math.max(deepestDropRpm, rpmDrop);
                if (deepestDropRpm < shotThreshold) {
                    detectionState = DetectionState.WAITING_FOR_DROP;
                    break;
                }

                double recoveredRise = deepestDropRpm - rpmDrop;
                if (rpmSlope >= Math.abs(ShooterConstants.autoShotDetectRecoverySlopeRpmPerSecond)
                        || recoveredRise >= recoveryRiseThreshold) {
                    detectedShots++;
                    detectionState = DetectionState.WAITING_FOR_RESET;
                }
                break;

            case WAITING_FOR_RESET:
                if (rpmDrop <= resetThreshold) {
                    detectionState = DetectionState.WAITING_FOR_DROP;
                    deepestDropRpm = 0.0;
                }
                break;
        }
    }

    public boolean isComplete() {
        return active && (detectedShots >= expectedShots
                || (burstWindowStarted
                && burstTimer.seconds() >= ShooterConstants.autoShotBurstTimeoutSeconds));
    }

    public int getDetectedShots() {
        return detectedShots;
    }

    public boolean isActive() {
        return active;
    }

    public double getFilteredRpm() {
        return filteredRpm;
    }

    public double getLastDropRpm() {
        return lastDropRpm;
    }

    public double getLastSlopeRpmPerSecond() {
        return lastSlopeRpmPerSecond;
    }

    public double getDeepestDropRpm() {
        return deepestDropRpm;
    }

    public String getDetectionStateName() {
        return detectionState.name();
    }
}
