package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "AprilTag TeleOp", group = "Vision")
public class AprilTagPipelinetele extends LinearOpMode {

    private AprilTagPipeline aprilTagPipeline;
    Relocalisation relocalisation = new Relocalisation(hardwareMap, aprilTagPipeline);

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing AprilTag Pipeline...");
        telemetry.update();

        // Instantiate and start the vision system
        aprilTagPipeline = new AprilTagPipeline(hardwareMap);
        aprilTagPipeline.startCamera();

        telemetry.addLine("Ready. Press Play to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            relocalisation.relocalisation();

            Pose currentPose = relocalisation.getPedroPose();

            // Get all current detections
            List<AprilTagDetection> detections = aprilTagPipeline.getAllDetections();

            if (detections.isEmpty()) {
                telemetry.addLine("No AprilTags Detected.");
            } else {
                telemetry.addData("Detections", detections.size());
                for (AprilTagDetection detection : detections) {
                    telemetry.addLine(String.format("ID: %d | ftcpose: (%.2f, %.2f)",
                            detection.id,
                            detection.ftcPose.x,
                            detection.ftcPose.y));
                }
            }

            if (currentPose != null) {
                telemetry.addData("X", currentPose.getX());
                telemetry.addData("Y", currentPose.getY());
                telemetry.addData("Heading", currentPose.getHeading());
            } else {
                telemetry.addData("Pose", "No detection yet");
            }

            telemetry.update();
        }
    }
}
