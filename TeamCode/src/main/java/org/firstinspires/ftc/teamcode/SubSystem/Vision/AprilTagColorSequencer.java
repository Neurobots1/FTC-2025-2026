package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AprilTagColorSequencer {

    /*AprilTagColorSequencer sequencer = new AprilTagColorSequencer();

    int detectedId = 21;
    String sequence = sequencer.getColorSequence(detectedId);
    System.out.println("Color sequence for tag " + detectedId + ": " + sequence);*/

    private AprilTagPipeline aprilTagPipeline;

    public String getColorSequence() {
        List<AprilTagDetection> detections = aprilTagPipeline.getAllDetections();

        for (AprilTagDetection detection : detections) {

            if (detection.metadata.id == 20) {
                return "blue";
            }

            if (detection.metadata.id == 21) {

                return "green purple purple";  // gpp
            }

            if (detection.metadata.id == 22) {

                return "purple purple green";  // ppg
            }

            if (detection.metadata.id == 23) {

                return "purple purple green";  // ppg
            }

            if (detection.metadata.id == 34) {

                return "red";
            }
        }
        return "unknown";
    }
}
