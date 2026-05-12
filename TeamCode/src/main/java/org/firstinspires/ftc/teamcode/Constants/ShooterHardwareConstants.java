package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class ShooterHardwareConstants {

    public enum FlywheelFeedbackMotor {
        LEFT,
        RIGHT
    }

    private ShooterHardwareConstants() {
    }

    // ===== Names =====
    public static String leftFlywheelName = HardwareMapConstants.SHOOTER_LEFT_FLYWHEEL_MOTOR;
    public static String rightFlywheelName = HardwareMapConstants.SHOOTER_RIGHT_FLYWHEEL_MOTOR;
    public static String hoodServoName = HardwareMapConstants.HOOD_SERVO;
    public static String turretMotorName = HardwareMapConstants.TURRET_MOTOR;
    public static String feedServoName = HardwareMapConstants.FEED_SERVO;

    // ===== Hardware Toggles =====
    public static boolean turretEnabled = true;

    // ===== Motor Direction =====
    public static boolean leftFlywheelReversed = true;
    public static boolean rightFlywheelReversed = false;
    public static FlywheelFeedbackMotor flywheelFeedbackMotor = FlywheelFeedbackMotor.RIGHT;
    public static boolean flywheelFeedbackEncoderReversed = true;
    public static boolean turretMotorReversed = true;

    // ===== Hood Geometry =====
    public static double hoodMinAngleDeg = 30.0;
    public static double hoodMaxAngleDeg = 55.0;
    public static double hoodServoMinPosition = 1.0;
    public static double hoodServoMaxPosition = 0.0;
    public static double hoodCommandDeadbandDeg = 0.20;

    // ===== Turret Geometry =====
    public static double turretMechanicalRangeDeg = 355.0;
    public static double turretMechanicalRangeTicks = 1370;
    public static double turretCenterOffsetDeg = 0.0;
    public static double turretOffsetForwardInches = -1.0;
    public static double turretOffsetLeftInches = 0.0;
    public static double turretForbiddenCenterDeg = 0.0;
    public static double turretForbiddenWidthDeg = 0.0;

    // ===== Feed Servo =====
    public static double feedClosedPosition = 0.5;
    public static double feedOpenPosition = 0.12;
}
