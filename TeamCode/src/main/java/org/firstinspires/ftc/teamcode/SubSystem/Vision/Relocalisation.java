package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class Relocalisation {

    private final AprilTagPipeline aprilTagPipeline;

    // Pose from FTC AprilTag system (field coordinates)
    public Pose ftcpose;

    public Relocalisation(HardwareMap hw, AprilTagPipeline aprilTagPipeline) {
        this.aprilTagPipeline = aprilTagPipeline;
    }

    /**
     * Runs one relocalization step using AprilTag detections.
     * Returns pose in field coordinates if available, otherwise null.
     */
    public Pose relocalisation() {
        if (aprilTagPipeline == null) {
            return null;
        }

        List<AprilTagDetection> detections = aprilTagPipeline.getAllDetections();
        if (detections == null || detections.isEmpty()) {
            return null;
        }

        for (AprilTagDetection detection : detections) {

            if (detection.metadata == null) continue;
            if (detection.metadata.name != null && detection.metadata.name.contains("Obelisk")) continue;
            if (detection.ftcPose == null) continue;

            double x = detection.ftcPose.x;
            double y = detection.ftcPose.y;
            double headingDegrees = detection.ftcPose.yaw;

            ftcpose = new Pose(x, y, headingDegrees);

            return ftcpose;
        }

        return null;
    }
}
