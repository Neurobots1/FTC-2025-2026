package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision.PrecisionShotTable;

@Configurable
public class PrecisionShooterConfig {

    public static final double TICKS_PER_REV = 28.0;
    public static final double GRAVITY_INCHES_PER_SECOND_SQUARED = 386.0886;

    public String leftFlywheelName = "ShooterA";
    public String rightFlywheelName = "ShooterB";
    public String hoodServoName = "hood";
    public String turretMotorName = "turret";
    public String feedServoName = "Blocker";

    public boolean leftFlywheelReversed = true;
    public boolean rightFlywheelReversed = false;
    public boolean turretMotorReversed = false;

    public double nominalBatteryVoltage = 12.6;
    public double flywheelKp = 0.00055;
    public double flywheelKi = 0.00012;
    public double flywheelKd = 0.00001;
    public double flywheelKv = 0.00020;
    public double flywheelKs = 0.055;
    public double flywheelIntegralLimit = 5000.0;
    public double flywheelReadyToleranceRpm = 45.0;
    public double minCompensationRpmFraction = 0.82;

    public double hoodMinAngleDeg = 30.0;
    public double hoodMaxAngleDeg = 55.0;
    public double hoodServoMinPosition = 0.16;
    public double hoodServoMaxPosition = 0.79;
    public double hoodSettleSeconds = 0.18;

    public double turretMechanicalRangeDeg = 180.0;
    public double turretCenterOffsetDeg = 0.0;
    public double turretOffsetForwardInches = 0.0;
    public double turretOffsetLeftInches = 0.0;
    public double turretHomePower = 0.15;
    public double turretBackoffPower = 0.20;
    public double turretBackoffTicks = 80.0;
    public double turretHomeCurrentAmps = 4.2;
    public double turretHomeVelocityTicksPerSecond = 12.0;
    public double turretHomeHoldSeconds = 0.18;
    public double turretPositionKp = 2.3;
    public double turretPositionKd = 0.04;
    public double turretStatic = 0.05;
    public double turretAngleToleranceDeg = 0.35;
    public double turretMaxCommand = 0.65;

    public double feedClosedPosition = 0.22;
    public double feedOpenPosition = 0.58;
    public double feedActuationSeconds = 0.10;
    public double shotReleaseLatencySeconds = 0.11;

    public double shooterHeightInches = 12.0;
    public double targetHeightInches = 36.0;

    public double blueGoalXInches = 0.0;
    public double blueGoalYInches = 144.0;
    public double redGoalXInches = 144.0;
    public double redGoalYInches = 144.0;

    public PrecisionShotTable table = PrecisionShotTable.defaultTable();
}
