package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision.PrecisionShotTable;

@Configurable
public class PrecisionShooterConfig {

    public static final double TICKS_PER_REV = 28.0;
    public static final double GRAVITY_INCHES_PER_SECOND_SQUARED = 386.0886;

    public static String leftFlywheelName = "ShooterA";
    public static String rightFlywheelName = "ShooterB";
    public static String hoodServoName = "hood";
    public static String turretMotorName = "turret";
    public static String feedServoName = "Blocker";

    public static boolean turretEnabled = false;
    public static boolean lockChassisHeadingWhenTurretDisabled = true;

    public static boolean leftFlywheelReversed = true;
    public static boolean rightFlywheelReversed = false;
    public static boolean turretMotorReversed = false;

    public static double nominalBatteryVoltage = 12.40;
    public static double flywheelKp = 0.002256;
    public static double flywheelKi = 0.028205;
    public static double flywheelKd = 0.0000;
    public static double flywheelKv = 0.000169;
    public static double flywheelKs = 0.143503;
    public static double flywheelIntegralLimit = 5000.0;
    public static double flywheelReadyToleranceRpm = 45.0;
    public static double minCompensationRpmFraction = 0.82;

    public static double hoodMinAngleDeg = 30.0;
    public static double hoodMaxAngleDeg = 55.0;
    public static double hoodServoMinPosition = 1.0;
    public static double hoodServoMaxPosition = 0.0;
    public static double hoodSettleSeconds = 0.18;

    public static double turretMechanicalRangeDeg = 180.0;
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
    public static double turretAngleToleranceDeg = 0.35;
    public static double turretMaxCommand = 0.65;
    public static double chassisAimToleranceDeg = 2.0;
    public static double chassisAimKp = 1.6;
    public static double chassisAimMaxTurnCommand = 0.65;
    public static double chassisAimManualOverrideThreshold = 0.12;

    public static double feedClosedPosition = 0.22;
    public static double feedOpenPosition = 0.58;
    public static double feedActuationSeconds = 0.10;
    public static double shotReleaseLatencySeconds = 0.11;

    public static double shooterHeightInches = 12.0;
    public static double targetHeightInches = 36.0;

    public static double blueGoalXInches = 0.0;
    public static double blueGoalYInches = 144.0;
    public static double redGoalXInches = 144.0;
    public static double redGoalYInches = 144.0;

    // Panels can tune primitive fields live, so the shot table is exposed as rows instead
    // of a single complex object.
    public static double shot1DistanceInches = 36.0;
    public static double shot1TargetRpm = 2650.0;
    public static double shot1HoodAngleDeg = 52.0;

    public static double shot2DistanceInches = 48.0;
    public static double shot2TargetRpm = 2800.0;
    public static double shot2HoodAngleDeg = 49.0;

    public static double shot3DistanceInches = 60.0;
    public static double shot3TargetRpm = 2950.0;
    public static double shot3HoodAngleDeg = 46.0;

    public static double shot4DistanceInches = 72.0;
    public static double shot4TargetRpm = 3080.0;
    public static double shot4HoodAngleDeg = 43.5;

    public static double shot5DistanceInches = 84.0;
    public static double shot5TargetRpm = 3220.0;
    public static double shot5HoodAngleDeg = 40.5;

    public static double shot6DistanceInches = 96.0;
    public static double shot6TargetRpm = 3375.0;
    public static double shot6HoodAngleDeg = 38.0;

    public static double shot7DistanceInches = 108.0;
    public static double shot7TargetRpm = 3525.0;
    public static double shot7HoodAngleDeg = 35.5;

    public static double shot8DistanceInches = 120.0;
    public static double shot8TargetRpm = 3680.0;
    public static double shot8HoodAngleDeg = 33.0;

    public static PrecisionShotTable currentTable() {
        return PrecisionShotTable.fromArray(new double[][]{
                {shot1DistanceInches, shot1TargetRpm, shot1HoodAngleDeg},
                {shot2DistanceInches, shot2TargetRpm, shot2HoodAngleDeg},
                {shot3DistanceInches, shot3TargetRpm, shot3HoodAngleDeg},
                {shot4DistanceInches, shot4TargetRpm, shot4HoodAngleDeg},
                {shot5DistanceInches, shot5TargetRpm, shot5HoodAngleDeg},
                {shot6DistanceInches, shot6TargetRpm, shot6HoodAngleDeg},
                {shot7DistanceInches, shot7TargetRpm, shot7HoodAngleDeg},
                {shot8DistanceInches, shot8TargetRpm, shot8HoodAngleDeg}
        });
    }
}
