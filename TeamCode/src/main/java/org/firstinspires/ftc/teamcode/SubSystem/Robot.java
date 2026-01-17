package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.ConvertToPedroPose;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.HeadingLockController;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Launcher23511;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.Relocalisation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {

    public enum Alliance {
        BLUE,
        RED
    }

    private Alliance alliance = Alliance.BLUE;

    private Follower follower;
    private Launcher23511 launcher;
    private HeadingLockController headingLockController;
    private IntakeMotor intake;
    private AprilTagPipeline aprilTag;
    private Relocalisation relocalisation;
    private Indexer_Base indexerBase;

    // Shooter ready → rumble state
    private final ElapsedTime shooterReadyTimer = new ElapsedTime();
    private boolean headingLockWasEnabled = false;
    private boolean shooterReadyRumbleSent = false;

    private final Pose startingPose = new Pose(72, 72, Math.toRadians(90));

    public void init(HardwareMap hw) {

        aprilTag = new AprilTagPipeline(hw);
        aprilTag.startCamera();
        relocalisation = new Relocalisation(hw, aprilTag);

        follower = Constants.createFollower(hw);
        follower.update();

        intake = new IntakeMotor(hw);

        launcher = Launcher23511.create(hw);

        Servo blocker = hw.get(Servo.class, "Blocker");
        launcher.setBlocker(blocker);

        PIDFController pid = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        headingLockController = new HeadingLockController(pid, follower);

        // default goal as blue until alliance is set
        headingLockController.setGoalBlue();



        indexerBase = new Indexer_Base(hw);
        indexerBase.StartIndexPose();
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
                Pose pedroPose = tagPose;
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

        double distance = getDistanceToGoal();
        launcher.updateShooting(shootButton, pose.getX(), pose.getY(), distance);

        boolean headingLock = launcher.isHeadingLockEnabled();

        // ------------------------
        //  SHOOTER READY → RUMBLE
        //  one time per heading-lock cycle
        // ------------------------
        // Detect start/end of a shooting cycle via headingLock
        if (headingLock && !headingLockWasEnabled) {
            // just entered a shooting cycle (ARMING/FIRING)
            shooterReadyTimer.reset();
            shooterReadyRumbleSent = false;
        } else if (!headingLock && headingLockWasEnabled) {
            // exited shooting cycle (back to IDLE), allow rumble next time
            shooterReadyRumbleSent = false;
        }
        headingLockWasEnabled = headingLock;

        boolean shooterReady = launcher.flywheelReady();

        // Only rumble once per cycle, when shooter is ready and delay passed
        if (headingLock
                && shooterReady
                && !shooterReadyRumbleSent
                && shooterReadyTimer.milliseconds() > 150) {

            gamepad.rumble(300); // 0.3s rumble
            shooterReadyRumbleSent = true;
        }
        // ------------------------

        double manualTurn = -gamepad.right_stick_x;
        double turn = headingLockController.update(pose, headingLock, manualTurn);

        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                turn,
                false,
                Math.toRadians(180)
        );

        if (!indexerBase.isBusy()) {
            if (gamepad.right_bumper) intake.intake();
            else if (gamepad.left_bumper) intake.outtake();
            else intake.stop();
        }

        if (gamepad.dpad_down) {
            indexerBase.startOutTake();
        }

        indexerBase.OutTake();

        double targetTPS = Launcher23511.targetTPS;
        double currentVel = launcher.getCurentRPM();

        jt.addData("Alliance", alliance);
        jt.addData("targetTicksPerSecond", targetTPS);
        jt.addData("currentVelocity", "%.0f", currentVel);
        jt.addData("position", follower.getPose());
        jt.addData("Velocity", follower.getVelocity().getMagnitude());
        jt.addData("distance to goal", getDistanceToGoal());
        jt.update();
        telemetryManager.update();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        if (headingLockController != null) {
            if (alliance == Alliance.BLUE) {
                headingLockController.setGoalBlue();
            } else {
                headingLockController.setGoalRed();
            }
        }
    }

    public Alliance getAlliance() {
        return alliance;
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
        double gx = headingLockController.getGoalX();
        double gy = headingLockController.getGoalY();
        double dx = gx - follower.getPose().getX();
        double dy = gy - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }
}
