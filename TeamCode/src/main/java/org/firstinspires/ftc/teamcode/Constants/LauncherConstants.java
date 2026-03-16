package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class LauncherConstants {

    public static double P = 0.03;
    public static double I = 0.003;
    public static double D = 0.00004;
    public static double F = 0.000039;

    public static double VELOCITY_TOLERANCE = 60;

    public static double BLOCKER_OPEN_POSITION = 0.55;
    public static double BLOCKER_CLOSED_POSITION = 0.25;
    public static double BLOCKER_OPEN_DELAY_S = 0.3;

    public static double MAX_FLYWHEEL_VELOCITY = 1860;
    public static double NOMINAL_VOLTAGE = 12.82;

    public static boolean MOTOR_ONE_REVERSED = false;
    public static boolean MOTOR_TWO_REVERSED = true;

    public static double FAR_ZONE_TPS = 820;

    private LauncherConstants() {
    }
}
