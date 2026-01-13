package org.firstinspires.ftc.teamcode.Global;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {

    // ----------------------------
    // Flywheel PIDF coefficients
    // ----------------------------
    // Replace these with your tuned PIDF values
    public static final PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS =
            new PIDFCoefficients(
                    0.002,   // P
                    0.0,     // I
                    0.0002,  // D
                    0.0009   // F
            );

    // Velocity tolerance (ticks/sec) for atSetPoint()
    public static final double FLYWHEEL_VEL_TOLERANCE = 25.0;

    // ----------------------------
    // Flywheel behavior
    // ----------------------------
    // Max ticks/sec your shooter can reach
    public static final double LAUNCHER_MAX_VELOCITY = 2200.0;

    // When not using PID control (activeControl = false)
    // This is the default "spin-up" power
    public static final double LAUNCHER_DEFAULT_ON_SPEED = 0.75;

}
