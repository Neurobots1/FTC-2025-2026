package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Limelight Botpose TeleOp", group = "Test")
public class LimelightBotposeTeleOp extends LinearOpMode {

    private Limelight3A limelight;

    // Your field transform constants (same as your code)
    private static final double INCHES_PER_METER = 39.37007874015748; // 1 / 0.0254
    private static final double FIELD_OFFSET_IN = 70.625;

    private Pose lastGoodPose = null;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // If your SDK requires starting/stream enable, do it here.
        // Some versions use limelight.start(); others don't. If you get a compile error, tell me.
        // limelight.start();

        telemetry.addLine("Limelight ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Button edge detection ---
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            // A: example toggle (LED / pipeline stuff varies by SDK version)
            if (a && !lastA) {
                // If your Limelight SDK has LED controls, plug them here.
                // Example (may differ by version): limelight.setLEDMode(Limelight3A.LEDMode.ON);
                telemetry.addLine("A pressed (add LED toggle here if your SDK supports it)");
            }

            // Read latest result
            LLResult result = limelight.getLatestResult();

            Pose pose = null;
            boolean valid = (result != null && result.isValid());

            if (valid) {
                Pose3D robotPos = result.getBotpose();

                // Yaw + 90 like your original code
                double yawDeg = robotPos.getOrientation().getYaw(AngleUnit.DEGREES) + 90.0;
                yawDeg = (yawDeg % 360.0 + 360.0) % 360.0; // normalize 0..360

                // Convert meters to inches, then your field transform
                double xIn = (robotPos.getPosition().y * INCHES_PER_METER) + FIELD_OFFSET_IN;
                double yIn = (-robotPos.getPosition().x * INCHES_PER_METER) + FIELD_OFFSET_IN;
                double headingRad = Math.toRadians(yawDeg);

                pose = new Pose(xIn, yIn, headingRad);
            }

            // B: snapshot the last good pose
            if (b && !lastB) {
                if (pose != null) lastGoodPose = pose;
            }

            // --- Telemetry ---
            telemetry.addData("Result", (result == null) ? "null" : "non-null");
            telemetry.addData("Valid", valid);

            if (pose != null) {
                telemetry.addData("Pose X (in)", "%.2f", pose.getX());
                telemetry.addData("Pose Y (in)", "%.2f", pose.getY());
                telemetry.addData("Pose H (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
            } else {
                telemetry.addLine("Pose: (no valid target)");
            }

            if (lastGoodPose != null) {
                telemetry.addData("LastGood X", "%.2f", lastGoodPose.getX());
                telemetry.addData("LastGood Y", "%.2f", lastGoodPose.getY());
                telemetry.addData("LastGood H (deg)", "%.1f", Math.toDegrees(lastGoodPose.getHeading()));
            }

            telemetry.addLine("B = snapshot last good pose");
            telemetry.update();

            lastA = a;
            lastB = b;
        }

        // If you started it manually, stop it here.
        // limelight.stop();
    }
}
