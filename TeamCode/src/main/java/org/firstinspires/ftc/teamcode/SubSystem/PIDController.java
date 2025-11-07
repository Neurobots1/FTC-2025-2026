package org.firstinspires.ftc.teamcode.SubSystem;

public class PIDController {
    private double kP, kI, kD;
    private double integral, previousError;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0;
        this.previousError = 0;
    }

    public double calculate(double target, double current) {
        double error = target - current;
        integral += error;
        double derivative = error - previousError;
        previousError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    public void reset() {
        integral = 0;
        previousError = 0;
    }
}
