package org.firstinspires.ftc.teamcode.Subsystems;

import android.content.Context;
import android.content.SharedPreferences;

import com.pedropathing.geometry.Pose;

public final class AutoPoseHandoff {

    private static final String PREFS_NAME = "auto_pose_handoff";
    private static final String KEY_VALID = "valid";
    private static final String KEY_X = "x";
    private static final String KEY_Y = "y";
    private static final String KEY_HEADING = "heading";

    private AutoPoseHandoff() {
    }

    public static void savePose(Context context, Pose pose) {
        if (context == null || pose == null) {
            return;
        }

        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        prefs.edit()
                .putBoolean(KEY_VALID, true)
                .putLong(KEY_X, Double.doubleToRawLongBits(pose.getX()))
                .putLong(KEY_Y, Double.doubleToRawLongBits(pose.getY()))
                .putLong(KEY_HEADING, Double.doubleToRawLongBits(pose.getHeading()))
                .apply();
    }

    public static Pose consumePose(Context context) {
        if (context == null) {
            return null;
        }

        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        if (!prefs.getBoolean(KEY_VALID, false)) {
            return null;
        }

        Pose pose = new Pose(
                Double.longBitsToDouble(prefs.getLong(KEY_X, Double.doubleToRawLongBits(0.0))),
                Double.longBitsToDouble(prefs.getLong(KEY_Y, Double.doubleToRawLongBits(0.0))),
                Double.longBitsToDouble(prefs.getLong(KEY_HEADING, Double.doubleToRawLongBits(0.0)))
        );

        prefs.edit().clear().apply();
        return pose;
    }
}
