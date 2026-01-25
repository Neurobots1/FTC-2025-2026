package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




@Configurable
@TeleOp
public class AprilTagGB extends OpMode {

    private Follower follower;
    private AprilTagPipeline aprilTag;

    public static double MAX_RANGE_IN = 80;
    public static double MAX_JUMP_IN = 18;
    public static double MAX_HEADING_JUMP_DEG = 30;
    public static double ALPHA = 0.2;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        aprilTag = new AprilTagPipeline(hardwareMap);
        aprilTag.startCamera();
    }

    @Override
    public void loop() {
        follower.update();

        AprilTagPipeline.RobotPose ftcPose = aprilTag.getRobotFieldPose();
        Pose pedroRaw = aprilTag.getRobotFieldPosePedro();
        Pose pedroFiltered = aprilTag.getRobotFieldPosePedroFiltered(
                follower.getPose(),
                MAX_RANGE_IN,
                MAX_JUMP_IN,
                MAX_HEADING_JUMP_DEG,
                ALPHA
        );

        telemetry.addData("CameraActive", aprilTag.isCameraActive());

        if (ftcPose != null) {
            telemetry.addData("FTC_X", ftcPose.x);
            telemetry.addData("FTC_Y", ftcPose.y);
            telemetry.addData("FTC_H_deg", Math.toDegrees(ftcPose.headingRad));
            telemetry.addData("FTC_Tag", ftcPose.tagIdUsed);
        } else {
            telemetry.addData("FTC", "null");
        }

        if (pedroRaw != null) {
            telemetry.addData("PedroRaw_X", pedroRaw.getX());
            telemetry.addData("PedroRaw_Y", pedroRaw.getY());
            telemetry.addData("PedroRaw_H_deg", Math.toDegrees(pedroRaw.getHeading()));
        } else {
            telemetry.addData("PedroRaw", "null");
        }

        if (pedroFiltered != null) {
            telemetry.addData("PedroFilt_X", pedroFiltered.getX());
            telemetry.addData("PedroFilt_Y", pedroFiltered.getY());
            telemetry.addData("PedroFilt_H_deg", Math.toDegrees(pedroFiltered.getHeading()));
        } else {
            telemetry.addData("PedroFilt", "null");
        }

        Pose odo = follower.getPose();
        telemetry.addData("Odo_X", odo.getX());
        telemetry.addData("Odo_Y", odo.getY());
        telemetry.addData("Odo_H_deg", Math.toDegrees(odo.getHeading()));

        telemetry.update();
    }

    @Override
    public void stop() {
        if (aprilTag != null) {
            try { aprilTag.stopCamera(); } catch (Exception ignored) {}
        }
    }
}
