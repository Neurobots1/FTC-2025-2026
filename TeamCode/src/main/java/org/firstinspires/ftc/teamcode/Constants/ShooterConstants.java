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
    public static double flywheelReadyToleranceRpm = 90.0;
    public static double flywheelReadyToleranceFraction = 0.030;
    public static double flywheelSustainToleranceRpm = 150.0;
    public static double flywheelSustainToleranceFraction = 0.050;

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
    public static double preFeedIntakePower = 0.8;
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
    public static double turretShotReadyToleranceDeg = 4.0;
    public static double turretShotSustainToleranceDeg = 6.0;
    public static double turretLimitBufferDeg = 3.0;
    public static double turretLimitSlowZoneDeg = 18.0;
    public static double turretLimitMinScale = 0.18;
    public static double turretDeadZoneNudgeTurnCommand = 0.5;
    public static double turretDeadZoneNudgeDriverOverrideThreshold = 0.08;

    // ===== Chassis Aim PIDF =====
    public static double chassisAimKp = 0.6;
    public static double chassisAimKi = 0.0;
    public static double chassisAimKd = 0.14;
    public static double chassisAimKf = 0.05;

    // ===== Shot Ready =====
    public static double shotReadyHeadingToleranceDeg = 8.0;
    public static double shotReadyMaxOmegaDegPerSecond = 30.0;
    public static double shotSustainHeadingToleranceDeg = 11.0;
    public static double shotSustainMaxOmegaDegPerSecond = 40.0;
    public static double feedReadyDropoutGraceSeconds = 0.18;

    // ===== Driver Trim =====
    public static double aimTrimStepDeg = 0.35;
    public static double rpmTrimStep = 35.0;
    public static double trimAdjustCooldownSeconds = 0.10;

    // ===== Timing =====
    public static double feedOpenSettlingSeconds = 0.30;

    // ===== SOTM =====
    public static boolean shootOnMoveEnabled = true;
    public static double sotmScoreAngleDeg = -8.0;
    public static double sotmPassThroughRadiusInches = 5.0;
    public static double sotmLaunchSpeedToRpmSlope = 13.55;
    public static double sotmLaunchSpeedToRpmIntercept = 250.0;

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
