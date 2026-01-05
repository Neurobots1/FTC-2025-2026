package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class Relocalisationfilter {

    private final AprilTagPipeline aprilTagPipeline;

    public Pose ftcpose;
    public Pose kalmanPose;
    public Pose filteredPose;

    private static final double ALPHA = 0.25;
    private static final double INCHES_TO_MM = 25.4; // Si Pedro utilise mm

    private final KalmanFilter xFilter;
    private final KalmanFilter yFilter;
    private final KalmanFilter headingFilter;

    public Relocalisationfilter(HardwareMap hardwareMap) {
        this.aprilTagPipeline = new AprilTagPipeline(hardwareMap);

        KalmanFilterParameters params = new KalmanFilterParameters(2.0, 10.0);

        xFilter = new KalmanFilter(params, 0, 1, 1);
        yFilter = new KalmanFilter(params, 0, 1, 1);
        headingFilter = new KalmanFilter(params, 0, 1, 1);
    }

    public Pose relocalisation() {
        if (aprilTagPipeline == null) return filteredPose;

        List<AprilTagDetection> detections = aprilTagPipeline.getAllDetections();
        if (detections == null || detections.isEmpty()) return filteredPose;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata == null) continue;
            if (detection.metadata.name.contains("Obelisk")) continue;
            if (detection.robotPose == null) continue;

            // Conversion correcte des unités
            double x = detection.robotPose.getPosition().x * INCHES_TO_MM;
            double y = detection.robotPose.getPosition().y * INCHES_TO_MM;
            double heading = Math.toRadians(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));

            ftcpose = new Pose(x, y, heading);

            // --- STEP 1: Kalman Filter ---
            double prevX = (kalmanPose == null) ? x : kalmanPose.getX();
            double prevY = (kalmanPose == null) ? y : kalmanPose.getY();
            double prevH = (kalmanPose == null) ? heading : kalmanPose.getHeading();

            xFilter.update(prevX, x);
            double kX = xFilter.getState();

            yFilter.update(prevY, y);
            double kY = yFilter.getState();

            // Handle heading wrap-around
            double deltaH = heading - prevH;
            if (deltaH > Math.PI) deltaH -= 2 * Math.PI;
            if (deltaH < -Math.PI) deltaH += 2 * Math.PI;

            headingFilter.update(prevH, prevH + deltaH);
            double kHeading = headingFilter.getState();

            kalmanPose = new Pose(kX, kY, kHeading);

            // --- STEP 2: Low-pass filter ---
            if (filteredPose == null) {
                filteredPose = new Pose(kX, kY, kHeading);
            } else {
                double smoothX = filteredPose.getX() + ALPHA * (kX - filteredPose.getX());
                double smoothY = filteredPose.getY() + ALPHA * (kY - filteredPose.getY());

                double hDelta = kHeading - filteredPose.getHeading();
                if (hDelta > Math.PI) hDelta -= 2 * Math.PI;
                if (hDelta < -Math.PI) hDelta += 2 * Math.PI;

                double smoothHeading = filteredPose.getHeading() + ALPHA * hDelta;

                filteredPose = new Pose(smoothX, smoothY, smoothHeading);
            }

            return filteredPose;
        }

        return filteredPose;
    }
}