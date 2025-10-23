package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class Relocalisation extends OpenCvPipeline {

    private volatile AprilTagDetection latestDetection = null;
    public AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    private AprilTagPipeline aprilTagPipeline;
    private Pose ftcPose, pedroPose;

    // Constructor where you can initialize hardwareMap and pipeline
    public Relocalisation(HardwareMap hardwareMap, AprilTagPipeline aprilTagPipeline) {
        this.hardwareMap = hardwareMap;
        this.aprilTagPipeline = aprilTagPipeline;
        // Initialize aprilTag, visionPortal, or other components here if needed
    }

    public void relocalisation() {
        if (aprilTagPipeline == null) {
            // Pipeline not initialized; skip or throw exception
            return;
        }

        List<AprilTagDetection> detections = aprilTagPipeline.getAllDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);

            ftcPose = new Pose(
                    detection.ftcPose.x,
                    detection.ftcPose.y,
                    detection.ftcPose.yaw
            );

            pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }
    }

    public Pose getPedroPose() {
        return pedroPose;
    }

    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

}


     /*Relocalisation relocalisation = (new hardwareMap, aprilTagPipeline)
     while (OpModeIsActive) relocalisation.relocalisation();
    Pose currentPose = relocalisation.getPedroPose();
    if (currentPose != null) {
            telemetry.addData("X", currentPose.getX());
           telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
        }else {
       telemetry.addData("Pose", "No detection yet");
       }
        telemetry.update();
    } */

