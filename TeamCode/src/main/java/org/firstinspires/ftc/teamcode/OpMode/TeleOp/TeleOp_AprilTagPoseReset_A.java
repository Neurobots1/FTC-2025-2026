package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "TeleOp_AprilTagPoseReset", group = "TeleOp")
public class TeleOp_AprilTagPoseReset_A extends OpMode {

    private Follower follower;
    private AprilTagPipeline tagPipeline;

    private boolean lastA = false;

    // If true: updates pose whenever a valid tag is visible.
    // If false: updates only on A rising edge.
    private boolean CONTINUOUS = true;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        tagPipeline = new AprilTagPipeline(hardwareMap);
        tagPipeline.startCamera();

        follower.setPose(new Pose(72, 72, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        follower.update();

        // VisionPortal does NOT call OpenCvPipeline.processFrame(), so we poll detections.
        tagPipeline.update();

        AprilTagDetection latest = tagPipeline.getLatestDetection();

        telemetry.addData("Camera streaming", tagPipeline.isCameraActive());
        telemetry.addData("Detections", tagPipeline.getAllDetections().size());
        if (latest != null) {
            telemetry.addData("Latest ID", latest.id);
            if (latest.ftcPose != null) {
                telemetry.addData("ftcPose x(in)", latest.ftcPose.x);
                telemetry.addData("ftcPose y(in)", latest.ftcPose.y);
                telemetry.addData("ftcPose yaw(deg)", latest.ftcPose.yaw);
            }
            if (latest.metadata != null) {
                telemetry.addData("tagX raw", latest.metadata.fieldPosition.get(0));
                telemetry.addData("tagY raw", latest.metadata.fieldPosition.get(1));
            }
        }

        boolean a = gamepad1.a;

        boolean shouldUpdate = CONTINUOUS || (a && !lastA);
        if (shouldUpdate) {
            Pose newPose = estimatePedroPoseFromAprilTag(latest, follower.getPose());

            if (newPose != null) {
                follower.setPose(newPose);
                telemetry.addLine(CONTINUOUS ? "Pose updated from AprilTag (continuous)"
                        : "Pose updated from AprilTag (A pressed)");
                telemetry.addData("NEW x", newPose.getX());
                telemetry.addData("NEW y", newPose.getY());
                telemetry.addData("NEW headingDeg", Math.toDegrees(newPose.getHeading()));
            } else {
                telemetry.addLine("No valid AprilTag pose (need detection + metadata + ftcPose)");
            }
        }

        lastA = a;

        Pose p = follower.getPose();
        telemetry.addData("Pedro x", p.getX());
        telemetry.addData("Pedro y", p.getY());
        telemetry.addData("Pedro headingDeg", Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (tagPipeline != null) tagPipeline.stopCamera();
    }

    /**
     * Tag database is CENTER-origin (your tagX/tagY are negative), so Pedro conversion is always +72.
     * We compute multiple candidates (SDK conventions vary) and pick the one closest to odometry.
     */
    private Pose estimatePedroPoseFromAprilTag(AprilTagDetection det, Pose currentOdoPose) {
        if (det == null || det.ftcPose == null || det.metadata == null) return null;

        // Tag field position from database (VectorF), CENTER-origin inches
        double tagX = det.metadata.fieldPosition.get(0);
        double tagY = det.metadata.fieldPosition.get(1);

        // Tag yaw from quaternion (radians)
        double tagYaw = yawFromQuaternion(det.metadata.fieldOrientation);

        // FTC pose numbers (inches + degrees)
        // Your data (x~2.9, y~71.1) strongly indicates:
        //   y ~ forward/range, x ~ left/right
        double forward = det.ftcPose.y;
        double lr = det.ftcPose.x;

        // yaw relative (degrees->rad)
        double yawRel = Math.toRadians(det.ftcPose.yaw);

        Pose best = null;
        double bestScore = Double.POSITIVE_INFINITY;

        // Try:
        //  - left = +lr OR -lr
        //  - camHeading = tagYaw - yawRel OR tagYaw + yawRel
        double[] leftOptions = new double[]{ lr, -lr };
        double[] headingOptions = new double[]{
                wrapRad(tagYaw - yawRel),
                wrapRad(tagYaw + yawRel)
        };

        for (double left : leftOptions) {
            for (double camHeading : headingOptions) {

                // forward along heading, left is +90deg CCW from forward
                double camOffsetFieldX = forward * Math.cos(camHeading) - left * Math.sin(camHeading);
                double camOffsetFieldY = forward * Math.sin(camHeading) + left * Math.cos(camHeading);

                // camera position in CENTER-origin field coords
                double camX = tagX - camOffsetFieldX;
                double camY = tagY - camOffsetFieldY;

                // CENTER-origin -> PEDRO (corner-origin)
                double pedroX = toPedroCoord(camX);
                double pedroY = toPedroCoord(camY);

                // Reject if out of field
                if (!inField(pedroX, pedroY)) continue;

                // Score: prefer heading + position near current odometry
                double dHead = angleDiffRad(camHeading, currentOdoPose.getHeading());
                double dx = pedroX - currentOdoPose.getX();
                double dy = pedroY - currentOdoPose.getY();
                double dPos = Math.hypot(dx, dy);

                double score = dHead * 20.0 + dPos;

                if (score < bestScore) {
                    bestScore = score;
                    best = new Pose(pedroX, pedroY, camHeading);
                }
            }
        }

        return best;
    }

    private boolean inField(double x, double y) {
        // allow a little slack
        return x > -5 && x < 149 && y > -5 && y < 149;
    }

    // CONFIRMED: tag database is CENTER-origin => always shift +72
    private double toPedroCoord(double centerOrigin) {
        return centerOrigin + 72.0;
    }

    private double angleDiffRad(double a, double b) {
        return Math.abs(wrapRad(a - b));
    }

    private double wrapRad(double a) {
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        while (a > Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    private double yawFromQuaternion(Quaternion q) {
        if (q == null) return 0.0;

        double w = q.w;
        double x = q.x;
        double y = q.y;
        double z = q.z;

        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return Math.atan2(siny_cosp, cosy_cosp);
    }
}
