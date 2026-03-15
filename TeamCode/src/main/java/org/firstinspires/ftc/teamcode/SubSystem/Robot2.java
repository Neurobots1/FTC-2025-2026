package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.HeadingLockController;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot2 {

    public static boolean USE_LIMELIGHT = true;

    public enum Alliance {
        BLUE,
        RED
    }

    private Alliance alliance = Alliance.BLUE;

    private TurretSubsystem turret;
    private Follower follower;
    private LauncherSubsystem launcher;
    private HeadingLockController headingLockController;
    private IntakeMotor intake;
    private Indexer_Base indexerBase;

    private Limelight3A limelight;
    private Pose lastLimelightPose = null;

    private final ElapsedTime DpadUpTimer = new ElapsedTime();
    private final ElapsedTime DpadDownTimer = new ElapsedTime();

    private final Pose startingPose = new Pose(72, 72, Math.toRadians(90));

    private static final double INCHES_PER_METER = 39.37;
    private static final double FIELD_OFFSET_IN = 70.625;

    public void init(HardwareMap hw) {

        follower = Constants.createFollower(hw);
        follower.update();

        intake = new IntakeMotor(hw);

        launcher = LauncherSubsystem.create(hw);
        Servo blocker = hw.get(Servo.class, "Blocker");
        launcher.setBlocker(blocker);

        PIDFController headingPID = new PIDFController(
                new PIDFCoefficients(0.6, 0, 0.02, 0.02)
        );

        headingLockController = new HeadingLockController(
                headingPID,
                follower
        );

        turret = new TurretSubsystem(hw);

        turret.setGoal(
                headingLockController.getGoalX(),
                headingLockController.getGoalY()
        );

        applyAllianceGoal();

        indexerBase = new Indexer_Base(hw);
        indexerBase.StartIndexPose();

        if (USE_LIMELIGHT) {
            limelight = hw.get(Limelight3A.class, "limelight");

            try {
                limelight.start();
            } catch (Exception ignored) {}
        }
    }

    public void teleopLoop(Gamepad gamepad,
                           ElapsedTime tagResetTimer,
                           double tagCooldown,
                           JoinedTelemetry jt,
                           TelemetryManager telemetryManager) {

        follower.update();

        Pose currentPose = follower.getPose();

        turret.update(currentPose);

        launcher.update();

        boolean shootButton = gamepad.y;

        double distance = getDistanceToGoal();

        launcher.updateShooting(
                shootButton,
                currentPose.getX(),
                currentPose.getY(),
                distance
        );

        boolean headingLock = launcher.isHeadingLockEnabled();

        double manualTurn = -gamepad.right_stick_x;

        double turn = headingLockController.update(
                currentPose,
                headingLock,
                manualTurn
        );

        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                turn,
                false,
                0
        );

        Pose p = follower.getPose();

        jt.addData("X", p.getX());
        jt.addData("Y", p.getY());
        jt.addData("Heading", Math.toDegrees(p.getHeading()));

        jt.update();
        telemetryManager.update();
    }

    private Pose convertLimelightResultToPedroPose(LLResult result) {

        if (result == null || !result.isValid()) return null;

        Pose3D robotPos = result.getBotpose();
        if (robotPos == null) return null;

        double yawDeg =
                robotPos.getOrientation().getYaw(AngleUnit.DEGREES)
                        + 270;

        yawDeg = (yawDeg % 360 + 360) % 360;

        double xIn = (robotPos.getPosition().y * INCHES_PER_METER) + FIELD_OFFSET_IN;
        double yIn = (-robotPos.getPosition().x * INCHES_PER_METER) + FIELD_OFFSET_IN;

        return new Pose(xIn, yIn, Math.toRadians(yawDeg));
    }

    private void applyAllianceGoal() {

        if (headingLockController == null) return;

        if (alliance == Alliance.BLUE) {
            headingLockController.setGoalBlue();
        } else {
            headingLockController.setGoalRed();
        }
    }

    public double getDistanceToGoal() {

        double gx = headingLockController.getGoalX();
        double gy = headingLockController.getGoalY();

        double dx = gx - follower.getPose().getX();
        double dy = gy - follower.getPose().getY();

        return Math.hypot(dx, dy);
    }
}