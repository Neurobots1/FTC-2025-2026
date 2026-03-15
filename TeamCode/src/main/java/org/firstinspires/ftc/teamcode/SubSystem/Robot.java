package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.HeadingLockController;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {

    public static boolean USE_LIMELIGHT = true;

    public enum Alliance {
        BLUE,
        RED
    }

    private Alliance alliance = Alliance.BLUE;

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

    private static final double INCHES_PER_METER = 39.37007874015748;
    private static final double FIELD_OFFSET_IN = 70.625;

    public void init(HardwareMap hw) {

        follower = Constants.createFollower(hw);
        follower.update();

        intake = new IntakeMotor(hw);

        launcher = LauncherSubsystem.create(hw);
        Servo blocker = hw.get(Servo.class, "Blocker");
        launcher.setBlocker(blocker);

        PIDFController pid = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        headingLockController = new HeadingLockController(pid, follower);

        // Apply alliance goal AFTER creating controller
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

    public void startTeleop() {
        follower.startTeleopDrive();
        follower.setStartingPose(getAutoEndPose());

        if (USE_LIMELIGHT && limelight != null) {
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
        launcher.update();

        Pose currentPose = follower.getPose();

        //---------------- LIMELIGHT ----------------//

        Pose limelightPose = null;
        boolean limelightValid = false;
        LLResult llResult = null;

        if (USE_LIMELIGHT && limelight != null) {

            try {

                llResult = limelight.getLatestResult();
                limelightValid = (llResult != null && llResult.isValid());

                if (limelightValid) {

                    limelightPose = convertLimelightResultToPedroPose(llResult);

                    if (limelightPose != null) {
                        lastLimelightPose = limelightPose;
                    }
                }

            } catch (Exception ignored) {
                limelightValid = false;
            }
        }

        if (USE_LIMELIGHT
                && gamepad.b
                && tagResetTimer.seconds() >= tagCooldown
                && lastLimelightPose != null) {

            follower.setPose(lastLimelightPose);
            tagResetTimer.reset();
        }

        //-------------------------------------------//

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

        double fieldCentricOffset =
                AllianceSelector.Field.fieldCentricOffset(
                        alliance == Alliance.BLUE
                                ? AllianceSelector.Alliance.BLUE
                                : AllianceSelector.Alliance.RED
                );

        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                turn,
                false,
                fieldCentricOffset
        );

        //---------------- Intake ----------------//

        if (!indexerBase.isBusy()) {

            if (gamepad.right_bumper) intake.intake();
            else if (gamepad.left_bumper) intake.outtake();
            else intake.stop();
        }

        if (gamepad.dpad_left) {
            indexerBase.startOutTake();
        }

        //---------------- Goal adjust ----------------//

        if (gamepad.dpad_up && DpadUpTimer.seconds() >= 0.1) {

            double newX = headingLockController.getGoalX() + 35.0;
            headingLockController.setGoalX(newX);

            DpadUpTimer.reset();
        }

        if (gamepad.dpad_down && DpadDownTimer.seconds() >= 0.1) {

            double newX = headingLockController.getGoalX() - 10.0;
            headingLockController.setGoalX(newX);

            DpadDownTimer.reset();
        }

        //---------------- Pose reset ----------------//

        if (gamepad.options) {
            follower.setPose(getOptionsResetPose());
        }

        if (gamepad.share) {
            follower.setPose(startingPose);
        }

        indexerBase.OutTake();

        //---------------- Telemetry ----------------//

        Pose p = follower.getPose();

        jt.addData("Alliance", alliance);
        jt.addData("X", p.getX());
        jt.addData("Y", p.getY());
        jt.addData("Heading", Math.toDegrees(p.getHeading()));

        jt.addData("GoalX", headingLockController.getGoalX());
        jt.addData("GoalY", headingLockController.getGoalY());

        jt.addData("Distance To Goal", getDistanceToGoal());
        jt.addData("LL Enabled", USE_LIMELIGHT); jt.addData("LL Result", llResult == null ? "null" : "non-null");
        jt.addData("LL Result", llResult == null ? "null" : "non-null");
        jt.addData("LL Valid", limelightValid);

        jt.update();
        telemetryManager.update();
    }

    private Pose convertLimelightResultToPedroPose(LLResult result) {

        if (result == null || !result.isValid()) return null;

        Pose3D robotPos = result.getBotpose();
        if (robotPos == null) return null;

        double yawDeg =
                robotPos.getOrientation().getYaw(AngleUnit.DEGREES)
                        + 90.0
                        + 180;

        yawDeg = (yawDeg % 360 + 360) % 360;

        double xIn = (robotPos.getPosition().y * INCHES_PER_METER) + FIELD_OFFSET_IN;
        double yIn = (-robotPos.getPosition().x * INCHES_PER_METER) + FIELD_OFFSET_IN;

        return new Pose(xIn, yIn, Math.toRadians(yawDeg));
    }

    public void stop() {

        if (USE_LIMELIGHT && limelight != null) {

            try {
                limelight.stop();
            } catch (Exception ignored) {}
        }
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        applyAllianceGoal();
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

    private Pose getOptionsResetPose() {

        AllianceSelector.Alliance a =
                alliance == Alliance.BLUE
                        ? AllianceSelector.Alliance.BLUE
                        : AllianceSelector.Alliance.RED;

        double x = AllianceSelector.Field.resetX(a);
        double y = AllianceSelector.Field.resetY(a);

        return new Pose(x, y, Math.toRadians(270));
    }

    private Pose getAutoEndPose() {

        AllianceSelector.Alliance a =
                alliance == Alliance.BLUE
                        ? AllianceSelector.Alliance.BLUE
                        : AllianceSelector.Alliance.RED;

        double x = AllianceSelector.Field.EndAutoX(a);
        double y = AllianceSelector.Field.EndAutoY(a);

        double heading = Math.toRadians(
                AllianceSelector.Field.EndAutoH(a)
        );

        return new Pose(x, y, heading);
    }
}