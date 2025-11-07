package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.pedropathing.ftc.FTCCoordinates;

import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.Relocalisation;

import com.pedropathing.geometry.Pose;

@TeleOp(name = "AprilTag Relocalisation", group = "Test")
public class teleoptest extends OpMode {


    private Relocalisation relocalisation;
    private AprilTagPipeline aprilTagPipeline;

    @Override
    public void init() {

        // --- Initialize vision system ---
        aprilTagPipeline = new AprilTagPipeline(hardwareMap); // Replace with your pipeline setup
        aprilTagPipeline.startCamera();
        relocalisation = new Relocalisation(hardwareMap, aprilTagPipeline);

        telemetry.addLine("Initialized - Ready to start");
        telemetry.update();
    }

    @Override
    public void loop() {


        // --- Get and display localization info ---
        Pose currentPose = relocalisation.relocalisation();






        if (currentPose != null) {
            telemetry.addData("X (mm)", "%.2f", currentPose.getX());
            telemetry.addData("Y (mm)", "%.2f", currentPose.getY());
            telemetry.addData("Heading (deg)", "%.2f", currentPose.getHeading());




        }

        telemetry.update();
    }


}
