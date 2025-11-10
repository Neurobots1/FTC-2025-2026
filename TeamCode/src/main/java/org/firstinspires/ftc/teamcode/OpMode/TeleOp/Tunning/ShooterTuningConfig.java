package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tunning;



import com.bylazar.configurables.annotations.Configurable;

/** Edit these live in Panels (no buttons needed). */
@Configurable
public class ShooterTuningConfig {
    public static double P = 0.006;
    public static double I = 0.000;
    public static double D = 0.000;
    public static double T = 0.050;
    public static double F = 0.0005;

    /** TeleOp target RPM step sizes */
    public static double stepSmall = 25;
    public static double stepBig   = 100;

    /** Hood band servo positions (useful during LUT probing) */
    public static double HOOD_CLOSE = 0.72;
    public static double HOOD_MID   = 0.56;
    public static double HOOD_FAR   = 0.41;
}

