package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Collections;
import java.util.List;

public class AprilTagPipeline extends OpenCvPipeline {

    public AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private volatile AprilTagDetection latestDetection = null;
    private final HardwareMap hardwareMap;

    private final Position cameraPosition = new Position(DistanceUnit.INCH, 4.875, 8.4375, 10, 0);
    private final YawPitchRollAngles cameraorientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

    private volatile boolean closing = false;

    public AprilTagPipeline(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public synchronized void startCamera() {
        closing = false;

        if (visionPortal != null) {
            VisionPortal.CameraState st = visionPortal.getCameraState();
            if (st == VisionPortal.CameraState.STREAMING || st == VisionPortal.CameraState.STARTING_STREAM) {
                return;
            }
            safeClosePortal();
        }

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
                .enableLiveView(false)
                .build();

        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public synchronized void stopCamera() {
        if (closing) return;
        closing = true;

        latestDetection = null;

        if (visionPortal == null) {
            aprilTag = null;
            return;
        }

        try {
            if (aprilTag != null) {
                try {
                    visionPortal.setProcessorEnabled(aprilTag, false);
                } catch (RuntimeException ignored) { }
            }

            safeClosePortal();

        } finally {
            visionPortal = null;
            aprilTag = null;
        }
    }

    private void safeClosePortal() {
        try {
            if (visionPortal != null) {
                visionPortal.close();
            }
        } catch (RuntimeException ignored) {
        }
    }

    public boolean isCameraActive() {
        return visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }


    @Override
    public org.opencv.core.Mat processFrame(org.opencv.core.Mat input) {
        if (aprilTag == null) return input;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (!detections.isEmpty()) latestDetection = detections.get(0);
        return input;
    }

    public AprilTagDetection getLatestDetection() {
        return latestDetection;
    }

    public List<AprilTagDetection> getAllDetections() {
        if (aprilTag == null) return Collections.emptyList();
        return aprilTag.getDetections();
    }
}
