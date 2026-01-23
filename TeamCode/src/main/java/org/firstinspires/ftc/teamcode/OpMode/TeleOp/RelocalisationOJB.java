package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@TeleOp
public class RelocalisationOJB extends OpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Follower follower;

    /**
     * Camera position relative to robot center
     * Measure from the center of your robot to the camera lens
     * X = right, Y = forward, Z = up (in inches)
     */
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0); // TODO: Adjust these values for your robot

    /**
     * Camera orientation
     * Yaw = 0 if pointing forward, +90 if pointing left, -90 if pointing right
     * Pitch = -90 if camera is horizontal (typical)
     * Roll = 0 if upright, 180 if upside-down
     */
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0); // TODO: Adjust for your camera mounting

    @Override
    public void init() {
        // Initialize AprilTag processor with calibrated lens intrinsics
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644) // Your calibrated values
                .build();

        // Create vision portal with webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        visionPortal.setProcessorEnabled(aprilTag, true);

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(9, 111, 0)); // Set your starting pose

        telemetry.addData("Status", "Initialized with calibrated camera");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        // Get robot pose from AprilTag detection
        Pose cameraPose = getRobotPoseFromCamera();
        Pose odometryPose = follower.getPose();

        // Update follower position with AprilTag relocalization
        if (cameraPose != null) {
            follower.setPose(cameraPose);
        }

        // Telemetry - Show both odometry and camera poses
        telemetry.addLine("=== ODOMETRY POSE ===");
        telemetry.addData("Odom X", "%.2f in", odometryPose.getX());
        telemetry.addData("Odom Y", "%.2f in", odometryPose.getY());
        telemetry.addData("Odom Heading", "%.1f°", Math.toDegrees(odometryPose.getHeading()));

        telemetry.addLine("\n=== CAMERA POSE ===");
        if (cameraPose != null) {
            telemetry.addData("Camera X", "%.2f in", cameraPose.getX());
            telemetry.addData("Camera Y", "%.2f in", cameraPose.getY());
            telemetry.addData("Camera Heading", "%.1f°", Math.toDegrees(cameraPose.getHeading()));
        } else {
            telemetry.addData("Camera Status", "No AprilTag Detected");
        }

        telemetry.update();
    }


    private Pose getRobotPoseFromCamera() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.isEmpty()) {
            return null;
        }

        // Find the best detection (with metadata and robotPose)
        for (AprilTagDetection detection : currentDetections) {
            // Only use tags that have metadata (are in the tag library)
            if (detection.metadata == null) {
                continue;
            }

            // Skip Obelisk tags (they're not useful for localization)
            if (detection.metadata.name.contains("Obelisk")) {
                continue;
            }

            // Check if we have a valid robot pose
            if (detection.robotPose == null) {
                continue;
            }

            // Get robot position from the detection (SDK does all the math for us!)
            Position robotPosition = detection.robotPose.getPosition();
            YawPitchRollAngles robotOrientation = detection.robotPose.getOrientation();

            // Extract X, Y, and Yaw
            double x = robotPosition.x;
            double y = robotPosition.y;
            double yaw = robotOrientation.getYaw(AngleUnit.RADIANS);

            // Convert from FTC coordinates to Pedro Pathing coordinates
            Pose ftcPose = new Pose(x, y, yaw, FTCCoordinates.INSTANCE);
            Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            // Optional: Add filtering here using Pedro's built-in filters
            // KalmanFilter or LowPassFilter for smoother pose estimates

            telemetry.addData("Tag ID", detection.id);
            telemetry.addData("Tag Name", detection.metadata.name);
            telemetry.addData("FTC X", "%.2f", x);
            telemetry.addData("FTC Y", "%.2f", y);
            telemetry.addData("FTC Yaw", "%.1f°", Math.toDegrees(yaw));

            return pedroPose;
        }

        return null; // No valid detections
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}