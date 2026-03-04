package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightHelper {
    private final Limelight3A limelight;

    // "limelight" must match the device name in the Robot Controller configuration
    public LimelightHelper(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public Pose getRobotPosFromTarget() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D robotPos = result.getBotpose();

            double angle = robotPos.getOrientation().getYaw(AngleUnit.DEGREES) + 90;
            angle = (angle % 360 + 360) % 360; // normalize 0..360

            return new Pose(
                    robotPos.getPosition().y / 0.0254 + 70.625,
                    -robotPos.getPosition().x / 0.0254 + 70.625,
                    Math.toRadians(angle)
            );
        }
        return null;
    }
}
