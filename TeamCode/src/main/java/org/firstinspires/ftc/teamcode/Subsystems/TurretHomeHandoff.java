package org.firstinspires.ftc.teamcode.Subsystems;

import android.content.Context;
import android.content.SharedPreferences;

public final class TurretHomeHandoff {

    private static final String PREFS_NAME = "turret_home_handoff";
    private static final String KEY_VALID = "valid";
    private static final String KEY_LEFT_STOP = "left_stop";
    private static final String KEY_RIGHT_STOP = "right_stop";
    private static final String KEY_TIMESTAMP_MS = "timestamp_ms";
    private static final long MAX_AGE_MS = 5L * 60L * 1000L;

    public static final class Calibration {
        public final int leftStopTicks;
        public final int rightStopTicks;

        Calibration(int leftStopTicks, int rightStopTicks) {
            this.leftStopTicks = leftStopTicks;
            this.rightStopTicks = rightStopTicks;
        }
    }

    private TurretHomeHandoff() {
    }

    public static void saveCalibration(Context context, int leftStopTicks, int rightStopTicks) {
        if (context == null) {
            return;
        }

        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit()
                .putBoolean(KEY_VALID, true)
                .putInt(KEY_LEFT_STOP, leftStopTicks)
                .putInt(KEY_RIGHT_STOP, rightStopTicks)
                .putLong(KEY_TIMESTAMP_MS, System.currentTimeMillis())
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

        long savedAtMs = prefs.getLong(KEY_TIMESTAMP_MS, 0L);
        int leftStopTicks = prefs.getInt(KEY_LEFT_STOP, 0);
        int rightStopTicks = prefs.getInt(KEY_RIGHT_STOP, 0);
        prefs.edit().clear().apply();

        if (savedAtMs <= 0L || System.currentTimeMillis() - savedAtMs > MAX_AGE_MS) {
            return null;
        }

        return new Calibration(leftStopTicks, rightStopTicks);
    }

    public static void clear(Context context) {
        if (context == null) {
            return;
        }

        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit()
                .clear()
                .apply();
    }
}
