package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShotTable;

@Configurable
public class ShooterConstants {

    public enum FlywheelFeedbackMotor {
        LEFT,
        RIGHT
    }

    public static final double TICKS_PER_REV = 28.0;
    public static final double GRAVITY_INCHES_PER_SECOND_SQUARED = 386.0886;

    public static String leftFlywheelName = HardwareMapConstants.SHOOTER_LEFT_FLYWHEEL_MOTOR;
    public static String rightFlywheelName = HardwareMapConstants.SHOOTER_RIGHT_FLYWHEEL_MOTOR;
    public static String hoodServoName = HardwareMapConstants.HOOD_SERVO;
    public static String turretMotorName = HardwareMapConstants.TURRET_MOTOR;
    public static String feedServoName = HardwareMapConstants.FEED_SERVO;

    public static boolean turretEnabled = false;
    public static boolean lockChassisHeadingWhenTurretDisabled = true;

    public static boolean leftFlywheelReversed = false;
    public static boolean rightFlywheelReversed = true;
    public static FlywheelFeedbackMotor flywheelFeedbackMotor = FlywheelFeedbackMotor.LEFT;
    public static boolean flywheelFeedbackEncoderReversed = true;
    public static boolean turretMotorReversed = false;

    public static double nominalBatteryVoltage = 12.40;
    public static double flywheelKp = 0.001;
    public static double flywheelKi = 0.0000;


    
    public static double flywheelKd = 0.0000;
    public static double flywheelKv = 0.000179;
    public static double flywheelKs = 0.000172;
    public static double flywheelIntegralLimit = 5000.0;
    public static double flywheelReadyToleranceRpm = 45.0;
    public static double flywheelCompensationDropFilterGain = 0.65;
    public static double flywheelCompensationRecoveryFilterGain = 0.06;
    public static double flywheelCompensationCloseDeadbandRpm = 70.0;
    public static double flywheelCompensationFarDeadbandRpm = 20.0;
    public static double flywheelCompensationCloseDeadbandFraction = 0.03;
    public static double flywheelCompensationFarDeadbandFraction = 0.005;
    public static double minCompensationRpmFraction = 0.82;
    public static double flywheelCompensationCloseDistanceInches = 40.0;
    public static double flywheelCompensationFarDistanceInches = 145.0;
    public static double flywheelCompensationMaxHoodDeltaDeg = 10.0;
    public static double hoodCompensationCloseDegPerRpm = 0.0020;
    public static double hoodCompensationFarDegPerRpm = 0.0075;
    public static double flywheelAutoTuneSweepMinSeconds = 1.5;
    public static double flywheelAutoTuneSweepMaxSeconds = 4.5;
    public static double flywheelAutoTuneStepMinSeconds = 1.0;
    public static double flywheelAutoTuneStepMaxSeconds = 6.0;
    public static double flywheelAutoTuneStopTimeoutSeconds = 3.0;
    public static double flywheelAutoTuneStableWindowSeconds = 0.35;
    public static double flywheelAutoTuneStableRpmRate = 120.0;
    public static double flywheelAutoTuneStoppedRpm = 90.0;
    public static double flywheelAutoTuneStepPower = 0.62;

    public static double hoodMinAngleDeg = 30.0;
    public static double hoodMaxAngleDeg = 55.0;
    public static double hoodServoMinPosition = 1.0;
    public static double hoodServoMaxPosition = 0.0;
    public static double hoodCommandDeadbandDeg = 0.20;
    public static double hoodMaxDegreesPerSecond = 360.0;
    public static double hoodSettleSeconds = 0.08;

    public static double turretMechanicalRangeDeg = 359;
    public static double turretCenterOffsetDeg = 0.0;
    public static double turretOffsetForwardInches = 0.0;
    public static double turretOffsetLeftInches = 0.0;
    public static double turretHomePower = 0.15;
    public static double turretBackoffPower = 0.20;
    public static double turretBackoffTicks = 80.0;
    public static double turretHomeCurrentAmps = 4.2;
    public static double turretHomeVelocityTicksPerSecond = 12.0;
    public static double turretHomeHoldSeconds = 0.18;
    public static double turretPositionKp = 2.3;
    public static double turretPositionKd = 0.04;
    public static double turretStatic = 0.05;
    public static double turretAngleToleranceDeg = 0.8;
    public static double turretMaxCommand = 0.65;
    public static double chassisAimToleranceDeg = 5;
    public static double chassisAimKp = 0.55;
    public static double chassisAimKi = 0.0;
    public static double chassisAimKd = 0.08;
    public static double chassisAimKf = 0.03;
    public static double chassisAimMaxTurnCommand = 0.65;
    public static double chassisAimDeadbandDeg = 0.80;
    public static double chassisAimIntegralZoneDeg = 8.0;
    public static double chassisAimIntegralLimit = 0.25;
    public static double chassisAimTargetFilterGain = 0.35;
    public static double chassisAimMaxCommandStepPerSecond = 10.0;
    public static double chassisAimManualOverrideThreshold = 0.06;
    public static double aimTrimStepDeg = 0.35;
    public static double rpmTrimStep = 35.0;
    public static double trimAdjustCooldownSeconds = 0.10;

    public static double feedClosedPosition = 0.5;
    public static double feedOpenPosition = 0.12;
    public static double feedActuationSeconds = 0.10;
    public static double feedOpenSettlingSeconds = 0.30;
    public static double shotReleaseLatencySeconds = 0.11;
    public static boolean useEmpiricalShotTableHood = true;

    public static double shooterHeightInches = 12.0;
    public static double targetHeightInches = 36.0;

    public static double blueGoalXInches = 0.0;
    public static double blueGoalYInches = 144.0;
    public static double redGoalXInches = 144.0;
    public static double redGoalYInches = 144.0;
    public static double blueHeadingAimXInches = 12.0;
    public static double blueHeadingAimYInches = 132.0;
    public static double redHeadingAimXInches = 132.0;
    public static double redHeadingAimYInches = 132.0;

    // Panels can tune primitive fields live, so the shot table is exposed as rows instead
    // of a single complex object.
    public static double shot1DistanceInches = 34.2417;
    public static double shot1TargetRpm = 2450.0;
    public static double shot1HoodAngleDeg = 30.00;

    public static double shot2DistanceInches = 58.9609;
    public static double shot2TargetRpm = 2725.0;
    public static double shot2HoodAngleDeg = 38.00;

    public static double shot3DistanceInches = 75.9841;
    public static double shot3TargetRpm = 2950.00;
    public static double shot3HoodAngleDeg = 40.25;

    public static double shot4DistanceInches = 91.4179;
    public static double shot4TargetRpm = 3150.0;
    public static double shot4HoodAngleDeg = 43.25;

    public static double shot5DistanceInches = 107.1952;
    public static double shot5TargetRpm = 3400.0;
    public static double shot5HoodAngleDeg = 46.5;

    public static double shot6DistanceInches = 121.2883;
    public static double shot6TargetRpm = 3625.0;
    public static double shot6HoodAngleDeg = 49.75;

    public static double shot7DistanceInches = 139.67;
    public static double shot7TargetRpm = 4175.0;
    public static double shot7HoodAngleDeg = 55.00;

    public static double shot8DistanceInches = 154.7216;
    public static double shot8TargetRpm = 4300.0;
    public static double shot8HoodAngleDeg = 55.0;

    public static double shot9DistanceInches = 0.0;
    public static double shot9TargetRpm = 0.0;
    public static double shot9HoodAngleDeg = 0.0;

    public static double shot10DistanceInches = 0.0;
    public static double shot10TargetRpm = 0.0;
    public static double shot10HoodAngleDeg = 0.0;

    public static double shot11DistanceInches = 0.0;
    public static double shot11TargetRpm = 0.0;
    public static double shot11HoodAngleDeg = 0.0;

    public static double shot12DistanceInches = 0.0;
    public static double shot12TargetRpm = 0.0;
    public static double shot12HoodAngleDeg = 0.0;

    public static PrecisionShotTable currentTable() {
        return PrecisionShotTable.fromArray(new double[][]{
                {shot1DistanceInches, shot1TargetRpm, shot1HoodAngleDeg},
                {shot2DistanceInches, shot2TargetRpm, shot2HoodAngleDeg},
                {shot3DistanceInches, shot3TargetRpm, shot3HoodAngleDeg},
                {shot4DistanceInches, shot4TargetRpm, shot4HoodAngleDeg},
                {shot5DistanceInches, shot5TargetRpm, shot5HoodAngleDeg},
                {shot6DistanceInches, shot6TargetRpm, shot6HoodAngleDeg},
                {shot7DistanceInches, shot7TargetRpm, shot7HoodAngleDeg},
                {shot8DistanceInches, shot8TargetRpm, shot8HoodAngleDeg},
                {shot9DistanceInches, shot9TargetRpm, shot9HoodAngleDeg},
                {shot10DistanceInches, shot10TargetRpm, shot10HoodAngleDeg},
                {shot11DistanceInches, shot11TargetRpm, shot11HoodAngleDeg},
                {shot12DistanceInches, shot12TargetRpm, shot12HoodAngleDeg}
        });
    }
}
