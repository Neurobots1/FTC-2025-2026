package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShotFeedCadenceController;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public final class ColorSensorDataLogger {
    private static final String LOG_TAG = "ShotPatternLogger";
    private static final String LOG_DIRECTORY_NAME = "shot-training";
    private static final String FILE_PREFIX = "auto_shot_training_";
    private static final double OBJECT_DETECTED_DISTANCE_MM = 20.0;
    private static final int FLUSH_INTERVAL_ROWS = 25;

    private final RevColorSensorV3 colorSensor;
    private final ElapsedTime runtimeTimer = new ElapsedTime();
    private final ElapsedTime sampleTimer = new ElapsedTime();

    private BufferedWriter writer;
    private File outputFile;
    private boolean logging;
    private int sampleIndex;
    private int rowsSinceFlush;
    private int pendingShotLabels;
    private int pendingResetLabels;
    private String lastError = "";

    public ColorSensorDataLogger(RevColorSensorV3 colorSensor) {
        this.colorSensor = colorSensor;
    }

    public void startSession() {
        close();
        lastError = "";

        try {
            File directory = new File(AppUtil.FIRST_FOLDER, LOG_DIRECTORY_NAME);
            if (!directory.exists() && !directory.mkdirs()) {
                throw new IOException("Failed to create log directory: " + directory.getAbsolutePath());
            }

            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            outputFile = new File(directory, FILE_PREFIX + timestamp + ".csv");
            writer = new BufferedWriter(new FileWriter(outputFile, false));
            writer.write(buildHeader());
            writer.newLine();
            writer.flush();

            logging = true;
            sampleIndex = 0;
            rowsSinceFlush = 0;
            pendingShotLabels = 0;
            pendingResetLabels = 0;
            runtimeTimer.reset();
            sampleTimer.reset();
            RobotLog.ii(LOG_TAG, "Logging shot training data to %s", outputFile.getAbsolutePath());
        } catch (IOException e) {
            lastError = e.getMessage() == null ? "Unknown I/O error" : e.getMessage();
            logging = false;
            writer = null;
            outputFile = null;
            RobotLog.ee(LOG_TAG, "Failed to start shot logger: %s", lastError);
        }
    }

    public void markManualShot() {
        pendingShotLabels++;
    }

    public void markCounterReset() {
        pendingResetLabels++;
    }

    public void logSample(PrecisionShooterSubsystem.TelemetrySnapshot snapshot,
                          ShotFeedCadenceController feedController,
                          boolean gateOpen) {
        if (!logging || writer == null || snapshot == null || feedController == null) {
            return;
        }

        boolean manualShotLabel = pendingShotLabels > 0;
        boolean manualResetLabel = pendingResetLabels > 0;
        if (manualShotLabel) {
            pendingShotLabels--;
        }
        if (manualResetLabel) {
            pendingResetLabels--;
        }

        double dtSeconds = Math.max(1e-3, sampleTimer.seconds());
        sampleTimer.reset();

        ColorSample colorSample = readColorSample();
        double rawRpmDrop = Math.max(0.0, snapshot.targetRpm - snapshot.actualRpm);

        try {
            writer.write(String.format(
                    Locale.US,
                    "%d,%d,%.6f,%.6f,%s,%s,%.3f,%.3f,%.3f,%s,%s,%s,%s,%s,%s,%.3f,%.6f,%.3f,%.3f,%s,%s,%.3f,%s,%s,%.6f,%.6f,%.6f,%.6f,%.3f,%.6f,%.6f",
                    sampleIndex++,
                    System.currentTimeMillis(),
                    runtimeTimer.seconds(),
                    dtSeconds,
                    boolString(manualShotLabel),
                    boolString(manualResetLabel),
                    snapshot.targetRpm,
                    snapshot.actualRpm,
                    rawRpmDrop,
                    boolString(feedController.isArmed()),
                    feedController.getState().name(),
                    boolString(gateOpen),
                    boolString(feedController.isBlockerSettledOpen()),
                    boolString(feedController.shouldForceFeedIntake()),
                    boolString(feedController.isFarZoneFeedPaused()),
                    boolString(snapshot.ready),
                    boolString(snapshot.inShootingZone),
                    snapshot.tableDistanceInches,
                    snapshot.timeOfFlightSeconds,
                    snapshot.compensationRpm,
                    snapshot.compensatedHoodDeg,
                    safeCsv(snapshot.alliance == null ? "" : snapshot.alliance.name()),
                    safeCsv(snapshot.status),
                    colorSample.distanceMm,
                    boolString(colorSample.objectDetected),
                    safeCsv(colorSample.detectedColor),
                    colorSample.red,
                    colorSample.green,
                    colorSample.blue,
                    colorSample.alpha,
                    colorSample.hueDeg,
                    colorSample.saturation,
                    colorSample.value
            ));
            writer.newLine();
            rowsSinceFlush++;
            if (rowsSinceFlush >= FLUSH_INTERVAL_ROWS || manualShotLabel || manualResetLabel) {
                writer.flush();
                rowsSinceFlush = 0;
            }
        } catch (IOException e) {
            lastError = e.getMessage() == null ? "Unknown I/O error" : e.getMessage();
            logging = false;
            RobotLog.ee(LOG_TAG, "Failed while writing shot logger sample: %s", lastError);
            closeWriterQuietly();
        }
    }

    public void close() {
        if (writer != null) {
            try {
                writer.flush();
                writer.close();
            } catch (IOException e) {
                lastError = e.getMessage() == null ? "Unknown I/O error" : e.getMessage();
                RobotLog.ee(LOG_TAG, "Failed to close shot logger: %s", lastError);
            }
        }

        writer = null;
        logging = false;
    }

    public boolean isLogging() {
        return logging;
    }

    public boolean hasColorSensor() {
        return colorSensor != null;
    }

    public String getOutputPath() {
        return outputFile == null ? "" : outputFile.getAbsolutePath();
    }

    public String getOutputFileName() {
        return outputFile == null ? "" : outputFile.getName();
    }

    public String getLastError() {
        return lastError;
    }

    private ColorSample readColorSample() {
        if (colorSensor == null) {
            return ColorSample.empty();
        }

        try {
            double distanceMm = colorSensor.getDistance(DistanceUnit.MM);
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float[] hsv = new float[3];
            Color.RGBToHSV(
                    clampColor(colors.red),
                    clampColor(colors.green),
                    clampColor(colors.blue),
                    hsv
            );

            return new ColorSample(
                    distanceMm,
                    distanceMm < OBJECT_DETECTED_DISTANCE_MM,
                    determineColor(hsv[0], hsv[1], hsv[2]),
                    colors.red,
                    colors.green,
                    colors.blue,
                    colors.alpha,
                    hsv[0],
                    hsv[1],
                    hsv[2]
            );
        } catch (RuntimeException e) {
            lastError = e.getMessage() == null ? "Color sensor read failed" : e.getMessage();
            RobotLog.ee(LOG_TAG, "Color sensor read failed: %s", lastError);
            return ColorSample.empty();
        }
    }

    private static int clampColor(float colorChannel) {
        return Math.max(0, Math.min(255, Math.round(colorChannel * 255.0f)));
    }

    private static String determineColor(float hue, float saturation, float value) {
        if (value < 0.01f || saturation < 0.05f) {
            return "None";
        }

        if (hue >= 15.0f && hue < 55.0f) {
            return "Red";
        }

        if (hue >= 60.0f && hue < 100.0f) {
            return "Yellow";
        }

        if (hue >= 200.0f && hue < 300.0f) {
            return "Blue";
        }

        return "None";
    }

    private static String buildHeader() {
        return "sample_index,wall_time_ms,opmode_time_sec,loop_dt_sec,manual_shot_label,manual_reset_label,"
                + "target_rpm,actual_rpm,raw_drop_rpm,feed_armed,"
                + "feed_state,gate_open,blocker_settled_open,force_feed_intake,far_zone_feed_paused,"
                + "shooter_ready,in_shooting_zone,table_distance_inches,time_of_flight_sec,compensation_rpm,"
                + "hood_deg,alliance,status,distance_mm,object_detected,detected_color,red_norm,green_norm,"
                + "blue_norm,alpha_norm,hue_deg,saturation,value";
    }

    private static String boolString(boolean value) {
        return value ? "1" : "0";
    }

    private static String safeCsv(String value) {
        if (value == null) {
            return "";
        }

        if (!value.contains(",") && !value.contains("\"")) {
            return value;
        }

        return "\"" + value.replace("\"", "\"\"") + "\"";
    }

    private void closeWriterQuietly() {
        if (writer == null) {
            return;
        }

        try {
            writer.close();
        } catch (IOException ignored) {
        }
        writer = null;
    }

    private static final class ColorSample {
        final double distanceMm;
        final boolean objectDetected;
        final String detectedColor;
        final double red;
        final double green;
        final double blue;
        final double alpha;
        final double hueDeg;
        final double saturation;
        final double value;

        ColorSample(double distanceMm,
                    boolean objectDetected,
                    String detectedColor,
                    double red,
                    double green,
                    double blue,
                    double alpha,
                    double hueDeg,
                    double saturation,
                    double value) {
            this.distanceMm = distanceMm;
            this.objectDetected = objectDetected;
            this.detectedColor = detectedColor;
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.alpha = alpha;
            this.hueDeg = hueDeg;
            this.saturation = saturation;
            this.value = value;
        }

        static ColorSample empty() {
            return new ColorSample(
                    Double.NaN,
                    false,
                    "",
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN
            );
        }
    }
}
