package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;

public class Relocalisation {
    private final AprilTagPipeline aprilTagPipeline;

    public Relocalisation(AprilTagPipeline aprilTagPipeline) {
        this.aprilTagPipeline = aprilTagPipeline;
    }

    public AprilTagPoseFtc relocalisationFtcPose() {
        if (aprilTagPipeline == null) return null;

        List<AprilTagDetection> detections = aprilTagPipeline.getAllDetections();
        if (detections == null || detections.isEmpty()) return null;

        for (AprilTagDetection detection : detections) {
            if (detection == null) continue;
            if (detection.metadata == null) continue;
            if (detection.metadata.name != null && detection.metadata.name.contains("Obelisk")) continue;
            if (detection.ftcPose == null) continue;

            return detection.ftcPose; // x,y inches, yaw degrees
        }
        return null;
    }
}
