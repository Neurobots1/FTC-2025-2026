package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.Relocalisation;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "AprilTag Relocalisation", group = "Test")
public class teleoptest extends OpMode {

    private Relocalisation relocalisation;
    private AprilTagPipeline aprilTagPipeline;
    private ConvertToPedroPose convertToPedroPose;

    @Override
    public void init() {
        // Initialize vision system
        aprilTagPipeline = new AprilTagPipeline(hardwareMap); // Replace with your pipeline setup
        aprilTagPipeline.startCamera();

        relocalisation = new Relocalisation(hardwareMap, aprilTagPipeline);

        // Initialize conversion utility
        convertToPedroPose = new ConvertToPedroPose();

        telemetry.addLine("Initialized - Ready to start");
        telemetry.update();
    }

    @Override
    public void loop() {
        Pose currentPose = relocalisation.relocalisation();
        Pose pedroPose = null;

        if (currentPose != null && convertToPedroPose != null) {
            pedroPose = convertToPedroPose.convertToPedroPose(currentPose);
        }

        telemetry.addLine("=== AprilTag FTC Relocalisation ===");
        if (currentPose != null) {
            telemetry.addData("Current X (mm)", "%.2f", currentPose.getX());
            telemetry.addData("Current Y (mm)", "%.2f", currentPose.getY());
            telemetry.addData("Current Heading (deg)", "%.2f", currentPose.getHeading());
        } else {
            telemetry.addLine("Current Pose: null");
        }

        telemetry.addLine();
        telemetry.addLine("=== PedroPose Conversion ===");
        if (pedroPose != null) {
            telemetry.addData("Pedro X (mm)", "%.2f", pedroPose.getX());
            telemetry.addData("Pedro Y (mm)", "%.2f", pedroPose.getY());
            telemetry.addData("Pedro Heading (deg)", "%.2f", pedroPose.getHeading());
        } else {
            telemetry.addLine("Pedro Pose: null");
        }

        telemetry.update();
    }
}





