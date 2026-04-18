package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.HardwareMapConstants;
import org.firstinspires.ftc.teamcode.Constants.PedroConstants;

@Configurable
@TeleOp(name = "POSE_FINDER", group = "Tuning")
public class PoseFinderTeleOp extends OpMode {

    public static double START_X = 20;
    public static double START_Y = 119;
    public static double START_HEADING_DEG = 139;

    private Follower follower;

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    private boolean previousA;

    @Override
    public void init() {
        follower = PedroConstants.createFollower(hardwareMap);
        cacheDriveMotors();
        setDriveMotorsFloat();
        applyConfiguredStartPose();
        follower.update();
    }

    @Override
    public void init_loop() {
        applyConfiguredStartPose();
        follower.update();
        addTelemetry("INIT");
    }

    @Override
    public void start() {
        applyConfiguredStartPose();
        follower.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a && !previousA) {
            applyConfiguredStartPose();
        }
        previousA = gamepad1.a;

        follower.update();
        addTelemetry("RUNNING");
    }

    @Override
    public void stop() {
        setDriveMotorPower(0.0);
    }

    private void applyConfiguredStartPose() {
        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
    }

    private void addTelemetry(String state) {
        Pose pose = follower.getPose();

        telemetry.addData("State", state);
        telemetry.addData("Start Pose", "(%.2f, %.2f, %.1f deg)", START_X, START_Y, START_HEADING_DEG);
        telemetry.addData("Pose X", "%.3f", pose.getX());
        telemetry.addData("Pose Y", "%.3f", pose.getY());
        telemetry.addData("Heading Deg", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Heading Rad", "%.4f", pose.getHeading());
        telemetry.addLine("Push the robot by hand to watch localization.");
        telemetry.addLine("A resets the pose back to the configured start pose.");
        telemetry.update();
    }

    private void cacheDriveMotors() {
        leftFront = hardwareMap.get(DcMotor.class, HardwareMapConstants.LEFT_FRONT_DRIVE_MOTOR);
        leftRear = hardwareMap.get(DcMotor.class, HardwareMapConstants.LEFT_REAR_DRIVE_MOTOR);
        rightFront = hardwareMap.get(DcMotor.class, HardwareMapConstants.RIGHT_FRONT_DRIVE_MOTOR);
        rightRear = hardwareMap.get(DcMotor.class, HardwareMapConstants.RIGHT_REAR_DRIVE_MOTOR);
    }

    private void setDriveMotorsFloat() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setDriveMotorPower(0.0);
    }

    private void setDriveMotorPower(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }
}
