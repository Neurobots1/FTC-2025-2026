package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDistanceCalculator {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private double distance = 0.0;
    private double horizontalAngle = 0.0;
    private double verticalAngle = 0.0;
    private int detectedTagId = -1;
    private boolean tagDetected = false;
    private Pose robotPose = null;// pedro pose
    public static double CameraPixel = 1000;  // (A modifier)
    public static double tagsize = 2.0; // Taille réelle du tag en pouces (ajuster selon ton tag)

    // Conversion d'unités
    public static double INCHES_TO_MM = 25.4;// quelle unite de mesure?
    // OU
    // public static double INCHES_TO_UNIT = 1.0; SI utilise des pouces

    public AprilTagDistanceCalculator(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();


        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    //Appeler dans la loop Teleop
    public void update() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections.isEmpty()) {
            tagDetected = false;
            distance = 0.0;
            detectedTagId = -1;
            robotPose = null;
            return;
        }

        AprilTagDetection bestDetection = getBestDetection(detections);

        if (bestDetection != null && bestDetection.ftcPose != null) {
            tagDetected = true;
            detectedTagId = bestDetection.id;

            // Données FTC
            distance = bestDetection.ftcPose.range;
            horizontalAngle = bestDetection.ftcPose.bearing;
            verticalAngle = bestDetection.ftcPose.elevation;

            // Convertir en Pedro Pose
            robotPose = convertToPedroPose(bestDetection);
        }
    }

    /**
     * Convertit la détection AprilTag en Pedro Pose
     */
    private Pose convertToPedroPose(AprilTagDetection detection) {
        if (detection.ftcPose == null) return null;


        double x_inches = detection.ftcPose.x;
        double y_inches = detection.ftcPose.y;
        double z_inches = detection.ftcPose.z;


        double x = x_inches * INCHES_TO_MM;  // A changer (* 1.0 si mesure en pouces)
        double y = y_inches * INCHES_TO_MM;
        double heading = Math.toRadians(detection.ftcPose.yaw);

        return new Pose(x, y, heading);
    }

    public Pose getPedroPose() {
        return robotPose;
    }

    public Pose getPedroPoseForTag(int tagId) {
        AprilTagDetection detection = findTagById(tagId);

        if (detection != null && detection.ftcPose != null) {
            return convertToPedroPose(detection);
        }

        return null;
    }

    public double calculateDistanceManually(AprilTagDetection detection) {
        if (detection == null || detection.corners == null || detection.corners.length < 4) {
            return 0.0;
        }


        double tagWidthPixels = Math.abs(detection.corners[1].x - detection.corners[0].x);


        double calculatedDistance = (tagsize * CameraPixel) / tagWidthPixels;

        return calculatedDistance;
    }

    private AprilTagDetection getBestDetection(List<AprilTagDetection> detections) {
        AprilTagDetection best = null;
        double bestDecisionMargin = 0.0;

        for (AprilTagDetection detection : detections) {
            if (detection.decisionMargin > bestDecisionMargin) {
                bestDecisionMargin = detection.decisionMargin;
                best = detection;
            }
        }

        return best;
    }

    public AprilTagDetection findTagById(int targetId) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == targetId) {
                return detection;
            }
        }

        return null;
    }

    public double getDistanceToTag(int tagId) {
        AprilTagDetection detection = findTagById(tagId);

        if (detection != null && detection.ftcPose != null) {
            return detection.ftcPose.range;
        }

        return -1.0; // tag pas trouve
    }

    public double[] getRobotPositionRelativeToTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (!detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);

            if (detection.ftcPose != null) {
                return new double[] {
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z
                };
            }
        }

        return new double[] {0, 0, 0};
    }

    public boolean isTagInRange(double minDistance, double maxDistance) {
        return tagDetected && distance >= minDistance && distance <= maxDistance;
    }


    public void stop() { // stop la camera
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public double getDistance() {
        return distance;
    }

    public double getDistanceInInches() {
        return distance;
    }

    public double getDistanceInCM() {
        return distance * 2.54;
    }

    public double getHorizontalAngle() {
        return horizontalAngle;
    }

    public double getVerticalAngle() {
        return verticalAngle;
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    public boolean isTagDetected() {
        return tagDetected;
    }

    public List<AprilTagDetection> getAllDetections() {
        return aprilTagProcessor.getDetections();
    }
}