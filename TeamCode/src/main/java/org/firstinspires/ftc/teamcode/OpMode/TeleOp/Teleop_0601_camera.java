package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.Relocalisation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;

//@TeleOp
public class Teleop_0601_camera extends OpMode {

    private JoinedTelemetry jt;
    private TelemetryManager telemetryManager;

    private Follower follower;
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(90));

    private AprilTagPipeline aprilTagPipeline;
    private Relocalisation relocalisation;

    private boolean lastA = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        jt = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        aprilTagPipeline = new AprilTagPipeline(hardwareMap);
        aprilTagPipeline.startCamera();
        relocalisation = new Relocalisation(aprilTagPipeline);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(startingPose);
    }

    @Override
    public void loop() {
        follower.update();

        List<AprilTagDetection> dets = aprilTagPipeline.getAllDetections();
        int tagCount = (dets == null) ? 0 : dets.size();
        telemetry.addData("Tag count", tagCount);

        boolean a = gamepad1.a;
        boolean aPressed = a && !lastA;
        lastA = a;

        if (aPressed) {
            AprilTagPoseFtc ftcPose = relocalisation.relocalisationFtcPose();
            if (ftcPose != null) {

                // --- FTC pose (corner-origin) -> center-origin ---
                double xCenter = ftcPose.x - 72.0;
                double yCenter = ftcPose.y - 72.0;
                double headingDeg = ftcPose.yaw;

                // --- 90° left rotation (CCW): (x, y) -> (-y, x) ---
                double xRot = -yCenter;
                double yRot =  xCenter;

                // --- center-origin -> Pedro field (72,72 is center) ---
                double xPedro = xRot + 72.0;
                double yPedro = yRot + 72.0;

                // --- heading: degrees -> apply frame rotation -> radians ---
                double headingPedroRad = Math.toRadians(headingDeg + 90.0);

                Pose newPose = new Pose(xPedro, yPedro, headingPedroRad);
                follower.setPose(newPose);

                telemetryManager.debug(
                        "Reloc",
                        "FTC(x=%.1f y=%.1f yaw=%.1f) -> Pedro(x=%.1f y=%.1f yaw=%.1f)",
                        ftcPose.x, ftcPose.y, ftcPose.yaw,
                        xPedro, yPedro, Math.toDegrees(headingPedroRad)
                );

            } else {
                telemetryManager.debug("Reloc", "NO TAG");
            }
        }

        jt.addData("Current Position", follower.getPose());

        jt.update();
        telemetryManager.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (aprilTagPipeline != null) {
            aprilTagPipeline.stopCamera();
        }
    }
}
