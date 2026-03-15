package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;

public class TurretSubsystem {

    private DcMotorEx turretMotor;

    private double goalX;
    private double goalY;

    private double kP = 0.01;

    public TurretSubsystem(HardwareMap hw) {

        turretMotor = hw.get(DcMotorEx.class, "turret");

        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setGoal(double x, double y) {

        goalX = x;
        goalY = y;
    }

    public void update(Pose robotPose) {

        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();

        double targetAngle = Math.atan2(dy, dx);

        double robotHeading = robotPose.getHeading();

        double turretError = normalize(targetAngle - robotHeading);

        double power = turretError * kP;

        turretMotor.setPower(power);
    }

    private double normalize(double angle) {

        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;

        return angle;
    }
}