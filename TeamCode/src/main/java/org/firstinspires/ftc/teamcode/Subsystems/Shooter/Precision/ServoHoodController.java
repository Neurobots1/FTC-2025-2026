package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

final class ServoHoodController {

    private final Servo servo;
    private final ShooterConstants config;
    private final ElapsedTime settleTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double commandedAngleDeg;
    private double appliedAngleDeg;

    ServoHoodController(Servo servo, ShooterConstants config) {
        this.servo = servo;
        this.config = config;
        commandedAngleDeg = config.hoodMaxAngleDeg;
        appliedAngleDeg = commandedAngleDeg;
        servo.setPosition(angleToServo(appliedAngleDeg));
        settleTimer.reset();
        loopTimer.reset();
    }

    void setAngleDegrees(double angleDeg) {
        double clamped = ShooterMath.clamp(angleDeg, config.hoodMinAngleDeg, config.hoodMaxAngleDeg);
        if (Math.abs(clamped - commandedAngleDeg) > config.hoodCommandDeadbandDeg) {
            commandedAngleDeg = clamped;
            settleTimer.reset();
        }
    }

    void update() {
        double dt = Math.max(1e-3, loopTimer.seconds());
        loopTimer.reset();
        double maxStep = config.hoodMaxDegreesPerSecond * dt;
        double delta = commandedAngleDeg - appliedAngleDeg;
        if (Math.abs(delta) > maxStep) {
            appliedAngleDeg += Math.signum(delta) * maxStep;
        } else {
            appliedAngleDeg = commandedAngleDeg;
        }
        servo.setPosition(angleToServo(appliedAngleDeg));
    }

    boolean isSettled() {
        return settleTimer.seconds() >= config.hoodSettleSeconds;
    }

    private double angleToServo(double angleDeg) {
        double t = (angleDeg - config.hoodMinAngleDeg) / (config.hoodMaxAngleDeg - config.hoodMinAngleDeg);
        return ShooterMath.lerp(config.hoodServoMinPosition, config.hoodServoMaxPosition, t);
    }
}
