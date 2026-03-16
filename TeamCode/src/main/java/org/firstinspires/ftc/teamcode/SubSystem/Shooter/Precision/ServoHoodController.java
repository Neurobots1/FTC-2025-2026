package org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.PrecisionShooterConfig;

final class ServoHoodController {

    private final Servo servo;
    private final PrecisionShooterConfig config;
    private final ElapsedTime settleTimer = new ElapsedTime();
    private double commandedAngleDeg;

    ServoHoodController(Servo servo, PrecisionShooterConfig config) {
        this.servo = servo;
        this.config = config;
        setAngleDegrees(config.hoodMaxAngleDeg);
        settleTimer.reset();
    }

    void setAngleDegrees(double angleDeg) {
        double clamped = ShooterMath.clamp(angleDeg, config.hoodMinAngleDeg, config.hoodMaxAngleDeg);
        if (Math.abs(clamped - commandedAngleDeg) > 1e-6) {
            commandedAngleDeg = clamped;
            settleTimer.reset();
        }
        servo.setPosition(angleToServo(clamped));
    }

    void update() {
        servo.setPosition(angleToServo(commandedAngleDeg));
    }

    boolean isSettled() {
        return settleTimer.seconds() >= config.hoodSettleSeconds;
    }

    private double angleToServo(double angleDeg) {
        double t = (angleDeg - config.hoodMinAngleDeg) / (config.hoodMaxAngleDeg - config.hoodMinAngleDeg);
        return ShooterMath.lerp(config.hoodServoMinPosition, config.hoodServoMaxPosition, t);
    }
}
