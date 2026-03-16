package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.Pose;

public final class CompetitionTeleopConstants {

    public static final double INCHES_PER_METER = 39.37007874015748;
    public static final double FIELD_OFFSET_IN = 70.625;

    public static final Pose DRIVER_START_POSE = new Pose(72, 72, Math.toRadians(90));

    public static final double BLUE_GOAL_X_IN = 0.0;
    public static final double BLUE_GOAL_Y_IN = 140.0;
    public static final double RED_GOAL_X_IN = 140.0;
    public static final double RED_GOAL_Y_IN = 140.0;

    public static final double GOAL_X_STEP_UP_IN = 35.0;
    public static final double GOAL_X_STEP_DOWN_IN = 10.0;
    public static final double GOAL_ADJUST_COOLDOWN_S = 0.10;

    public static final int LIMELIGHT_PIPELINE = 0;

    private CompetitionTeleopConstants() {
    }
}
