package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class CameraConstants {

    public static boolean LIMELIGHT_ENABLED = true;

    public static int TELEOP_LIMELIGHT_PIPELINE = 0;
    public static int AUTO_TAG_LIMELIGHT_PIPELINE = 1;

    public static double LIMELIGHT_POSE_FILTER_GAIN = 0.35;

    public static int SORT_TAG_GPP_ID = 21;
    public static int SORT_TAG_PGP_ID = 22;
    public static int SORT_TAG_PPG_ID = 23;

    private CameraConstants() {
    }
}
