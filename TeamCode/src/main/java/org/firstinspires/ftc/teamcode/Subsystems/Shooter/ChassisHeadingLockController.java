package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;

public final class ChassisHeadingLockController {
    private final ElapsedTime timer = new ElapsedTime();

    private double integral;
    private double previousError;
    private double lastTurnCommand;
    private double lastProportionalTerm;
    private double lastIntegralTerm;
    private double lastDerivativeTerm;
    private double lastFeedforwardTerm;

    public double update(PrecisionShooterSubsystem shooter,
                         PrecisionShooterSubsystem.TelemetrySnapshot shooterSnapshot) {
        if (shooter == null || shooterSnapshot == null || !shooterSnapshot.solutionValid) {
            reset();
            return 0.0;
        }

        double dt = Math.max(1e-3, timer.seconds());
        timer.reset();

        double error = shooter.getAdjustedChassisHeadingErrorRadians();
        lastProportionalTerm = error * ShooterConstants.chassisAimKp;
        lastDerivativeTerm = ((error - previousError) / dt) * ShooterConstants.chassisAimKd;
        lastFeedforwardTerm = Math.abs(error) > 1e-4
                ? Math.signum(error) * ShooterConstants.chassisAimKf
                : 0.0;

        double candidateIntegral = integral + error * dt;
        double candidateIntegralTerm = candidateIntegral * ShooterConstants.chassisAimKi;
        double unclampedCommand = lastProportionalTerm
                + candidateIntegralTerm
                + lastDerivativeTerm
                + lastFeedforwardTerm;

        if (Math.abs(unclampedCommand) <= 1.0
                || Math.signum(unclampedCommand) != Math.signum(error)) {
            integral = candidateIntegral;
        }

        lastIntegralTerm = integral * ShooterConstants.chassisAimKi;
        previousError = error;
        lastTurnCommand = clamp(
                lastProportionalTerm
                        + lastIntegralTerm
                        + lastDerivativeTerm
                        + lastFeedforwardTerm
        );
        return lastTurnCommand;
    }

    public void reset() {
        integral = 0.0;
        previousError = 0.0;
        lastTurnCommand = 0.0;
        lastProportionalTerm = 0.0;
        lastIntegralTerm = 0.0;
        lastDerivativeTerm = 0.0;
        lastFeedforwardTerm = 0.0;
        timer.reset();
    }

    public double getLastTurnCommand() {
        return lastTurnCommand;
    }

    public double getLastProportionalTerm() {
        return lastProportionalTerm;
    }

    public double getLastIntegralTerm() {
        return lastIntegralTerm;
    }

    public double getLastDerivativeTerm() {
        return lastDerivativeTerm;
    }

    public double getLastFeedforwardTerm() {
        return lastFeedforwardTerm;
    }

    private double clamp(double command) {
        if (command > 1.0) {
            return 1.0;
        }
        if (command < -1.0) {
            return -1.0;
        }
        return command;
    }
}
