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
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvPipeline;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;


import java.util.Collections;
import java.util.List;

public class AprilTagPipeline extends OpenCvPipeline {

    public AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private volatile AprilTagDetection latestDetection = null;
    private final HardwareMap hardwareMap;

    private final Position cameraPosition = new Position(DistanceUnit.INCH, -4.875, 8.4375, 10, 0);
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
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
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

    public static class RobotPose {
        public final double x;
        public final double y;
        public final double headingRad;
        public final int tagIdUsed;

        public RobotPose(double x, double y, double headingRad, int tagIdUsed) {
            this.x = x;
            this.y = y;
            this.headingRad = headingRad;
            this.tagIdUsed = tagIdUsed;
        }
    }

    public RobotPose getRobotFieldPose() {
        if (aprilTag == null) return null;

        List<AprilTagDetection> dets;
        try {
            dets = aprilTag.getDetections();
        } catch (Exception e) {
            return null;
        }
        if (dets == null || dets.isEmpty()) return null;

        for (AprilTagDetection d : dets) {
            if (d != null && d.robotPose != null) {
                double x = d.robotPose.getPosition().x;
                double y = d.robotPose.getPosition().y;
                double headingRad = Math.toRadians(d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                return new RobotPose(x, y, headingRad, d.id);
            }
        }

        return null;
    }

    public Pose getRobotFieldPosePedro() {
        RobotPose p = getRobotFieldPose();
        if (p == null) return null;
        return new Pose(p.x, p.y, p.headingRad, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    private Pose getRobotPoseFromCamera() {
        Pose p = getRobotFieldPosePedro();
        if (p == null) return null;
        return p;
    }

    public Pose getRobotFieldPosePedroFiltered(Pose currentPedroPose,
                                               double maxRangeIn,
                                               double maxJumpIn,
                                               double maxHeadingJumpDeg,
                                               double alpha) {
        if (aprilTag == null) return null;

        List<AprilTagDetection> dets;
        try {
            dets = aprilTag.getDetections();
        } catch (Exception e) {
            return null;
        }
        if (dets == null || dets.isEmpty()) return null;

        AprilTagDetection best = null;
        double bestScore = -1;

        for (AprilTagDetection d : dets) {
            if (d == null || d.robotPose == null || d.ftcPose == null) continue;

            double range = d.ftcPose.range;
            if (range <= 0 || range > maxRangeIn) continue;

            double score = 1.0 / Math.max(1e-6, range);
            if (score > bestScore) {
                bestScore = score;
                best = d;
            }
        }

        if (best == null) return null;

        double x = best.robotPose.getPosition().x;
        double y = best.robotPose.getPosition().y;
        double h = Math.toRadians(best.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));

        Pose visionPedro = new Pose(x, y, h, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        if (currentPedroPose == null) return visionPedro;

        double dx = visionPedro.getX() - currentPedroPose.getX();
        double dy = visionPedro.getY() - currentPedroPose.getY();
        double jump = Math.hypot(dx, dy);

        double dh = wrapRad(visionPedro.getHeading() - currentPedroPose.getHeading());
        double dhDeg = Math.abs(Math.toDegrees(dh));

        if (jump > maxJumpIn || dhDeg > maxHeadingJumpDeg) return null;

        double fx = lerp(currentPedroPose.getX(), visionPedro.getX(), alpha);
        double fy = lerp(currentPedroPose.getY(), visionPedro.getY(), alpha);
        double fh = angleLerp(currentPedroPose.getHeading(), visionPedro.getHeading(), alpha);

        return new Pose(fx, fy, fh, PedroCoordinates.INSTANCE);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double wrapRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double angleLerp(double a, double b, double t) {
        return a + wrapRad(b - a) * t;
    }

}
