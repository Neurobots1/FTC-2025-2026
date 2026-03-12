package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Limelight Botpose TeleOp", group = "Test")
public class LimelightBotposeTeleOp extends OpMode {

    private Limelight3A limelight;

    private static final double INCHES_PER_METER = 39.37007874015748;
    private static final double FIELD_OFFSET_IN = 70.625;

    private Pose lastGoodPose = null;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Put your AprilTag pipeline index here
        // Change 0 if your tag pipeline is on another slot
        try {
            limelight.pipelineSwitch(0);
        } catch (Exception e) {
            telemetry.addData("PipelineSwitch", "Not supported: %s", e.getMessage());
        }

        telemetry.addLine("Limelight mapped");
        telemetry.addLine("Press Start");
        telemetry.update();
    }

    @Override
    public void start() {
        try {
            limelight.start();
        } catch (Exception e) {
            telemetry.addData("Start", "Not supported: %s", e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;

        if (a && !lastA) {
            try {
                limelight.pipelineSwitch(0); // re-force pipeline if needed
            } catch (Exception ignored) {}
        }

        LLResult result = limelight.getLatestResult();

        Pose pose = null;
        boolean valid = result != null && result.isValid();

        if (valid) {
            try {
                Pose3D robotPos = result.getBotpose();

                if (robotPos != null) {
                    double yawDeg = robotPos.getOrientation().getYaw(AngleUnit.DEGREES) + 90.0;
                    yawDeg = (yawDeg % 360.0 + 360.0) % 360.0;

                    double xIn = (robotPos.getPosition().y * INCHES_PER_METER) + FIELD_OFFSET_IN;
                    double yIn = (-robotPos.getPosition().x * INCHES_PER_METER) + FIELD_OFFSET_IN;
                    double headingRad = Math.toRadians(yawDeg);

                    pose = new Pose(xIn, yIn, headingRad);
                }
            } catch (Exception e) {
                telemetry.addData("Botpose Error", e.getMessage());
            }
        }

        if (b && !lastB && pose != null) {
            lastGoodPose = pose;
        }

        telemetry.addData("Result", result == null ? "null" : "non-null");
        telemetry.addData("Valid", valid);

        try {
            if (result != null) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
            }
        } catch (Exception ignored) {}

        if (pose != null) {
            telemetry.addData("Pose X (in)", "%.2f", pose.getX());
            telemetry.addData("Pose Y (in)", "%.2f", pose.getY());
            telemetry.addData("Pose H (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
        } else {
            telemetry.addLine("Pose: no valid target");
        }

        if (lastGoodPose != null) {
            telemetry.addData("LastGood X", "%.2f", lastGoodPose.getX());
            telemetry.addData("LastGood Y", "%.2f", lastGoodPose.getY());
            telemetry.addData("LastGood H (deg)", "%.1f", Math.toDegrees(lastGoodPose.getHeading()));
        }

        telemetry.addLine("A = reselect pipeline");
        telemetry.addLine("B = snapshot pose");
        telemetry.update();

        lastA = a;
        lastB = b;
    }

    @Override
    public void stop() {
        try {
            limelight.stop();
        } catch (Exception ignored) {}
    }
}
