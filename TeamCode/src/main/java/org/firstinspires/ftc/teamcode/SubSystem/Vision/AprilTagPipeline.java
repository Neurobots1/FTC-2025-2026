package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import android.icu.text.Transliterator;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.firstinspires.ftc.vision.VisionPortal;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class AprilTagPipeline extends OpenCvPipeline {

    public AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private volatile AprilTagDetection latestDetection = null;
    private HardwareMap hardwareMap;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 9, 8, 0);

    private YawPitchRollAngles cameraorientation = new YawPitchRollAngles(AngleUnit.DEGREES,0,-90,0, 0);



    public AprilTagPipeline(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;


        aprilTag = new AprilTagProcessorImpl.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();
    }

    public AprilTagPipeline() {
    }

    public void startCamera() {
        aprilTag = new AprilTagProcessor.Builder()

                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644)
                .setCameraPose(cameraPosition, cameraorientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(aprilTag)
                .enableLiveView(true)
                .build();

           visionPortal.setProcessorEnabled(aprilTag, true);
    }

    @Override
    public Mat processFrame(Mat input) {
        // Since your input is already grayscale, no need to convert.
        // This is just for visual feedback
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection tag : detections) {
            latestDetection = tag;

            // Draw a simple box around the tag (optional)
            Imgproc.rectangle(input,
                    new Point(tag.corners[0].x, tag.corners[0].y),
                    new Point(tag.corners[2].x, tag.corners[2].y),
                    new Scalar(255), 2);

            // Put tag ID on the image
            Imgproc.putText(input,
                    "ID: " + tag.id,
                    new Point(tag.center.x - 20, tag.center.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    new Scalar(255),
                    2);
        }

        return input;
    }

    public AprilTagDetection getLatestDetection() {
        return latestDetection;
    }

    public List<AprilTagDetection> getAllDetections() {
        return aprilTag.getDetections();
    }
}


