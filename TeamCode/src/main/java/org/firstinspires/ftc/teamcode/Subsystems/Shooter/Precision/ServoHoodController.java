package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ShooterHardwareConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
final class ServoHoodController {

    private final Servo servo;
    private double commandedAngleDeg;

    ServoHoodController(Servo servo, ShooterConstants config) {
        this.servo = servo;
        commandedAngleDeg = ShooterHardwareConstants.hoodMaxAngleDeg;
        servo.setPosition(angleToServo(commandedAngleDeg));
    }

    void setAngleDegrees(double angleDeg) {
        double clamped = ShooterMath.clamp(angleDeg,
                ShooterHardwareConstants.hoodMinAngleDeg,
                ShooterHardwareConstants.hoodMaxAngleDeg);
        if (Math.abs(clamped - commandedAngleDeg) > ShooterHardwareConstants.hoodCommandDeadbandDeg) {
            commandedAngleDeg = clamped;
        }
    }

    void update() {
        servo.setPosition(angleToServo(commandedAngleDeg));
    }

    boolean isSettled() {
        return true;
    }

    private double angleToServo(double angleDeg) {
        double t = (angleDeg - ShooterHardwareConstants.hoodMinAngleDeg)
                / (ShooterHardwareConstants.hoodMaxAngleDeg - ShooterHardwareConstants.hoodMinAngleDeg);
        return ShooterMath.lerp(
                ShooterHardwareConstants.hoodServoMinPosition,
                ShooterHardwareConstants.hoodServoMaxPosition,
                t
        );
    }
}
