package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "AprilTag TeleOp", group = "Vision")
public class AprilTagPipelinetele extends LinearOpMode {

    private AprilTagPipeline aprilTagPipeline;
    private Relocalisation relocalisation;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing AprilTag Pipeline...");
        telemetry.update();

        // Instantiate and start the vision system
        aprilTagPipeline = new AprilTagPipeline(hardwareMap);
        aprilTagPipeline.startCamera();

        // Initialiser la relocalisation APRÈS que hardwareMap et aprilTagPipeline soient prêts
        relocalisation = new Relocalisation(hardwareMap, aprilTagPipeline);

        telemetry.addLine("AprilTag Pipeline Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTagPipeline.getAllDetections();

            if (detections == null || detections.isEmpty()) {
                telemetry.addLine("No AprilTags Detected.");
            } else {
                telemetry.addData("Detections", detections.size());
                for (AprilTagDetection detection : detections) {
                    if (detection.ftcPose != null) {
                        telemetry.addLine(String.format("ID: %d | Pose: (%.2f, %.2f)",
                                detection.id,
                                detection.ftcPose.x,
                                detection.ftcPose.y));
                    } else {
                        telemetry.addLine(String.format("ID: %d | Pose: unavailable", detection.id));
                    }
                }
            }

            telemetry.update();
        }
        }
    }