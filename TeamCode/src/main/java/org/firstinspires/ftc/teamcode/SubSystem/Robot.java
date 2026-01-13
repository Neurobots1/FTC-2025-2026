package org.firstinspires.ftc.teamcode.SubSystem;

import static org.firstinspires.ftc.teamcode.SubSystem.Shooter.HeadingLockController.GOAL_X;
import static org.firstinspires.ftc.teamcode.SubSystem.Shooter.HeadingLockController.GOAL_Y;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.ConvertToPedroPose;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.HeadingLockController;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Launcher23511;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.Relocalisation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {

    private Follower follower;
    private Launcher23511 launcher;
    private HeadingLockController headingLockController;
    private IntakeMotor intake;
    private AprilTagPipeline aprilTag;
    private Relocalisation relocalisation;

    public static double DEFAULT_TARGET_TPS = 400;

    private final Pose startingPose = new Pose(72, 72, Math.toRadians(90));

    public void init(HardwareMap hw) {
        follower = Constants.createFollower(hw);
        follower.update();

        intake = new IntakeMotor(hw);

        launcher = Launcher23511.create(hw);

        Servo blocker = hw.get(Servo.class, "Blocker");
        launcher.setBlocker(blocker);
        blocker.setPosition(0);

        PIDFController pid = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        headingLockController = new HeadingLockController(pid, follower);

        aprilTag = new AprilTagPipeline(hw);
        aprilTag.startCamera();
        relocalisation = new Relocalisation(hw, aprilTag);
    }

    public void startTeleop() {
        follower.startTeleopDrive();
        follower.setStartingPose(startingPose);
    }

    public void teleopLoop(Gamepad gamepad,
                           ElapsedTime tagResetTimer,
                           double tagCooldown,
                           JoinedTelemetry jt,
                           TelemetryManager telemetryManager) {

        follower.update();
        launcher.update();



        if (gamepad.options
                && follower.getVelocity().getMagnitude() < 0.3
                && tagResetTimer.seconds() > tagCooldown) {

            Pose tagPose = relocalisation.relocalisation();

            if (tagPose != null) {
                Pose pedroPose = ConvertToPedroPose.convertToPedroPose(tagPose);
                if (pedroPose != null) {
                    follower.setPose(pedroPose);
                    tagResetTimer.reset();
                }
            }
        } else {
            telemetryManager.addData("NePeuxReset", true);
        }

        Pose pose = follower.getPose();
        boolean shootButton = gamepad.y;

        launcher.updateShooting(shootButton, pose.getX(), pose.getY() );
        boolean headingLock = launcher.isHeadingLockEnabled();

        double manualTurn = -gamepad.right_stick_x;
        double turn = headingLockController.update(pose, headingLock, manualTurn);

        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                turn
        );

        if (gamepad.right_bumper) intake.intake();
        else if (gamepad.left_bumper) intake.outtake();
        else intake.stop();

        double targetTPS = launcher.getTargetTicksPerSecond();
        double currentVel = launcher.getCurrentVelocity();

        jt.addData("targetTicksPerSecond", "%.0f", targetTPS);
        jt.addData("currentVelocity", "%.0f", currentVel);
        jt.addData("position", follower.getPose());
        jt.addData("Velocity", follower.getVelocity().getMagnitude());
        jt.addData("distance to goal", getDistanceToGoal());
        jt.update();
        telemetryManager.update();
    }

    public Follower follower() {
        return follower;
    }

    public Launcher23511 launcher() {
        return launcher;
    }

    public HeadingLockController headingLockController() {
        return headingLockController;
    }

    public IntakeMotor intake() {
        return intake;
    }

    public Relocalisation relocalisation() {
        return relocalisation;
    }

    public AprilTagPipeline aprilTag() {
        return aprilTag;
    }

    public double getDistanceToGoal() {
        double dx = GOAL_X - follower.getPose().getX();
        double dy = GOAL_Y - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }
}
