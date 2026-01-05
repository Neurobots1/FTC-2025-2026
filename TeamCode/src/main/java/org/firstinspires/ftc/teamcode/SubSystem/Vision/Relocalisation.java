package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@Disabled
public class Relocalisation  /*extends AprilTagPipeline */{


        private final HardwareMap hardwareMap;
        private final AprilTagPipeline aprilTagPipeline;

        public Pose ftcpose;     // Pose from FTC AprilTag system (in field coords)
        public Pose pedroPose;   // Pose converted to Pedro coordinates (optional)

          public Relocalisation(HardwareMap hardwareMap, AprilTagPipeline aprilTagPipeline) {
            this.hardwareMap = hardwareMap;
            this.aprilTagPipeline = aprilTagPipeline;
        }

        /**
         * Runs one relocalization step using AprilTag detections.
         * Returns Pedro coordinate pose if available, otherwise null. **/

        public Pose relocalisation() {
            if (aprilTagPipeline == null) {
                return null;
            }

            List<AprilTagDetection> detections = aprilTagPipeline.getAllDetections();
            if (detections == null || detections.isEmpty()) {
                return null;
            }

            for (AprilTagDetection detection : detections) {

                // Skip detections with missing metadata or invalid names
                if (detection.metadata == null) continue;
                if (detection.metadata.name.contains("Obelisk")) continue;
                if (detection.robotPose == null) continue;

                // Get robot position and orientation in field space
                double x = detection.robotPose.getPosition().x;  // in inches
                double y = detection.robotPose.getPosition().y;  // in inches
                double headingDegrees = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Store FTC pose
                ftcpose = new Pose(x, y, headingDegrees);

                // Optionally, convert to Pedro coordinate system


                return ftcpose;

            }



            return null;

        }
}
