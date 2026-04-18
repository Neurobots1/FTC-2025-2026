package org.firstinspires.ftc.teamcode.Subsystems;

import android.content.Context;
import android.content.SharedPreferences;

public final class TurretHomeHandoff {

    private static final String PREFS_NAME = "turret_home_handoff";
    private static final String KEY_VALID = "valid";
    private static final String KEY_LEFT_STOP_TICKS = "left_stop_ticks";
    private static final String KEY_RIGHT_STOP_TICKS = "right_stop_ticks";

    public static final class Calibration {
        public final int leftStopTicks;
        public final int rightStopTicks;

        public Calibration(int leftStopTicks, int rightStopTicks) {
            this.leftStopTicks = leftStopTicks;
            this.rightStopTicks = rightStopTicks;
        }
    }

    private TurretHomeHandoff() {
    }

    public static void saveCalibration(Context context, int leftStopTicks, int rightStopTicks) {
        if (context == null || rightStopTicks <= leftStopTicks) {
            return;
        }

        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        prefs.edit()
                .putBoolean(KEY_VALID, true)
                .putInt(KEY_LEFT_STOP_TICKS, leftStopTicks)
                .putInt(KEY_RIGHT_STOP_TICKS, rightStopTicks)
                .apply();
    }

    public static Calibration consumeCalibration(Context context) {
        if (context == null) {
            return null;
        }

        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        if (!prefs.getBoolean(KEY_VALID, false)) {
            return null;
        }

        Calibration calibration = new Calibration(
                prefs.getInt(KEY_LEFT_STOP_TICKS, 0),
                prefs.getInt(KEY_RIGHT_STOP_TICKS, 0)
        );

        prefs.edit().clear().apply();
        return calibration.rightStopTicks > calibration.leftStopTicks ? calibration : null;
    }
}
