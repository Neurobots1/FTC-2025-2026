package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable
public final class TeleopConstants {
    public static final Pose DRIVER_START_POSE = new Pose(72, 72, Math.toRadians(90));

    public static final Pose BLUE_GOAL_POSE = new Pose(0.0, 144.0, 0.0);
    public static final Pose BLUE_HEADING_AIM_POSE = new Pose(
            ShooterConstants.blueHeadingAimXInches,
            ShooterConstants.blueHeadingAimYInches,
            0.0
    );
    public static boolean ALWAYS_SPIN_FLYWHEEL = true;
    public static boolean ENABLE_HEADING_LOCK = true;

    private TeleopConstants() {
    }
}
