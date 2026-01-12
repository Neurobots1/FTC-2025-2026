package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class Relocalisation {

    private final HardwareMap hardwareMap;
    private final AprilTagPipeline aprilTagPipeline;

    public Pose ftcpose;

    public Relocalisation(HardwareMap hardwareMap, AprilTagPipeline aprilTagPipeline) {
        this.hardwareMap = hardwareMap;
        this.aprilTagPipeline = aprilTagPipeline;
    }


    public Pose relocalisation() {
        if (aprilTagPipeline == null) return null;

        List<AprilTagDetection> detections = aprilTagPipeline.getAllDetections();
        if (detections == null || detections.isEmpty()) return null;

        for (AprilTagDetection detection : detections) {
            if (detection.robotPose == null) continue;   // robotPose needs tagLibrary + cameraPose

            double x = detection.robotPose.getPosition().x;
            double y = detection.robotPose.getPosition().y;
            double headingRad = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

            ftcpose = new Pose(x, y, headingRad);
            return ftcpose;
        }

        return null;
    }
}
