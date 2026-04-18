package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    public static final double TICKS_PER_REV = 28.0;
    public static final double GRAVITY_INCHES_PER_SECOND_SQUARED = 386.0886;

    public static boolean lockChassisHeadingWhenTurretDisabled = true;

    // ===== Flywheel PIDF =====
    public static double nominalBatteryVoltage = 12.40;
    public static double flywheelKp = 0.008;
    public static double flywheelKi = 0.0000;
    public static double flywheelKd = 0.0000;
    public static double flywheelKv = 0.000179;
    public static double flywheelKs = 0.000172;
    public static double flywheelIntegralLimit = 5000.0;

    // ===== Flywheel Ready =====
    public static double flywheelReadyToleranceRpm = 70.0;
    public static double flywheelReadyToleranceFraction = 0.025;

    // ===== Flywheel Recovery =====
    public static double flywheelRecoveryDeadbandRpm = 35.0;
    public static double flywheelRecoveryGain = 0.0012;
    public static double flywheelRecoveryMaxBoost = 0.18;
    public static double flywheelOutputRiseRatePerSecond = 12.0;
    public static double flywheelOutputFallRatePerSecond = 3.5;

    // ===== Flywheel Compensation =====
    public static double flywheelCompensationDropFilterGain = 0.65;
    public static double flywheelCompensationRecoveryFilterGain = 0.06;

    // ===== Far Zone Feed =====
    public static double farZoneFeedPauseDistanceInches = 130.0;
    public static double farZoneFeedPauseDropRpm = 140.0;
    public static double farZoneFeedPauseDropFraction = 0.035;
    public static double farZoneFeedResumeDropRpm = 130.0;
    public static double farZoneFeedResumeDropFraction = 0.031;

    // ===== Auto Feed Timing =====
    public static double autoShotFeedDurationSeconds = 1.5;
    public static double sortedShotFinishDelaySeconds = 0.30;

    // ===== Feed Intake =====
    public static double preFeedIntakePower = 0.6;
    public static double mainFeedIntakePower = 1;



    // ===== Turret =====
    public static double turretHomePower = 0.60;
    public static double turretBackoffPower = 0.20;
    public static double turretBackoffTicks = 80.0;
    public static double turretHomeCurrentAmps = 4.0;
    public static double turretHomeVelocityTicksPerSecond = 12.0;
    public static double turretHomeHoldSeconds = 0.18;
    public static double turretPositionKp = 0.7;
    public static double turretPositionKd = 1.5;
    public static double turretKf = 0.08;
    public static double turretControlToleranceDeg = 0.5;
    public static double turretShotReadyToleranceDeg = 2.5;
    public static double turretLimitBufferDeg = 3.0;
    public static double turretLimitSlowZoneDeg = 18.0;
    public static double turretLimitMinScale = 0.18;

    // ===== Chassis Aim PIDF =====
    public static double chassisAimKp = 0.6;
    public static double chassisAimKi = 0.0;
    public static double chassisAimKd = 0.14;
    public static double chassisAimKf = 0.05;

    // ===== Shot Ready =====
    public static double shotReadyHeadingToleranceDeg = 6.5;
    public static double shotReadyMaxOmegaDegPerSecond = 22.0;

    // ===== Driver Trim =====
    public static double aimTrimStepDeg = 0.35;
    public static double rpmTrimStep = 35.0;
    public static double trimAdjustCooldownSeconds = 0.10;

    // ===== Timing =====
    public static double feedOpenSettlingSeconds = 0.30;
    public static boolean shootOnMoveEnabled = false;
    public static double shotReleaseLatencySeconds = 0.11;

    // ===== Ballistics =====
    public static double shooterHeightInches = 12.0;
    public static double targetHeightInches = 36.0;

    // ===== Field Targets =====
    public static double blueGoalXInches = 0.0;
    public static double blueGoalYInches = 144.0;
    public static double redGoalXInches = 144.0;
    public static double redGoalYInches = 144.0;
    public static double blueHeadingAimXInches = 12.0;
    public static double blueHeadingAimYInches = 132.0;
    public static double redHeadingAimXInches = 132.0;
    public static double redHeadingAimYInches = 132.0;
}
