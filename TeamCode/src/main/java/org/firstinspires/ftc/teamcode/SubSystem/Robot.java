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
    private boolean lastValidLimelight = false;

    private final ElapsedTime DpadUpTimer = new ElapsedTime();
    private final ElapsedTime DpadDownTimer = new ElapsedTime();

    private final ElapsedTime shooterReadyTimer = new ElapsedTime();
    private boolean headingLockWasEnabled = false;
    private boolean shooterReadyRumbleSent = false;

    private final Pose startingPose = new Pose(72, 72, Math.toRadians(90));

    // Limelight constants
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
        headingLockController.setGoalBlue();

        indexerBase = new Indexer_Base(hw);
        indexerBase.StartIndexPose();

        if (USE_LIMELIGHT) {
            limelight = hw.get(Limelight3A.class, "limelight");

            try {
                limelight.start();
            } catch (Exception ignored) {
            }
        }
    }

    public void startTeleop() {
        follower.startTeleopDrive();
        follower.setStartingPose(getAutoEndPose());

        if (USE_LIMELIGHT && limelight != null) {
            try {
                limelight.start();
            } catch (Exception ignored) {
            }
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

        // ---------------- LIMELIGHT ----------------
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

        // Press B to reset follower pose from latest valid Limelight pose
        if (USE_LIMELIGHT
                && gamepad.b
                && tagResetTimer.seconds() >= tagCooldown
                && lastLimelightPose != null) {
            follower.setPose(lastLimelightPose);
            tagResetTimer.reset();
        }

        lastValidLimelight = limelightValid;
        // ------------------------------------------

        boolean shootButton = gamepad.y;
        double distance = getDistanceToGoal();
        launcher.updateShooting(shootButton, currentPose.getX(), currentPose.getY(), distance);

        boolean headingLock = launcher.isHeadingLockEnabled();

        if (headingLock && !headingLockWasEnabled) {
            shooterReadyTimer.reset();
            shooterReadyRumbleSent = false;
        } else if (!headingLock && headingLockWasEnabled) {
            shooterReadyRumbleSent = false;
        }
        headingLockWasEnabled = headingLock;

        boolean shooterReady = launcher.flywheelReady();

        if (headingLock
                && shooterReady
                && !shooterReadyRumbleSent
                && shooterReadyTimer.milliseconds() > 150) {
            gamepad.rumble(300);
            shooterReadyRumbleSent = true;
        }

        double manualTurn = -gamepad.right_stick_x;
        double turn = headingLockController.update(currentPose, headingLock, manualTurn);

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

        if (!indexerBase.isBusy()) {
            if (gamepad.right_bumper) intake.intake();
            else if (gamepad.left_bumper) intake.outtake();
            else intake.stop();
        }

        if (gamepad.dpad_left) {
            indexerBase.startOutTake();
        }

        if (gamepad.dpad_up && DpadUpTimer.seconds() >= 0.2) {
            double newY = headingLockController.getGoalY() + 3.0;
            headingLockController.setGoalY(newY);
            DpadUpTimer.reset();
        }

        if (gamepad.dpad_down && DpadDownTimer.seconds() >= 0.2) {
            double newY = headingLockController.getGoalY() - 3.0;
            headingLockController.setGoalY(newY);
            DpadDownTimer.reset();
        }

        if (gamepad.options) {
            follower.setPose(getOptionsResetPose());
        }

        if (gamepad.share) {
            follower.setPose(startingPose);
        }

        indexerBase.OutTake();

        jt.addData("Alliance", alliance);
        Pose p = follower.getPose();

        jt.addData("position",
                "x=%.2f y=%.2f h=%.2f",
                p.getX(),
                p.getY(),
                Math.toDegrees(p.getHeading()));
        jt.addData("Velocity", follower.getVelocity().getMagnitude());
        jt.addData("distance to goal", getDistanceToGoal());
        jt.addData("Shooter Target TPS", launcher.getTargetTPS());
        jt.addData("Shooter Current TPS", launcher.getCurrentRPM());
        jt.addData("GoalPoseY", headingLockController.getGoalY());

        jt.addData("LL Enabled", USE_LIMELIGHT);
        jt.addData("LL Result", llResult == null ? "null" : "non-null");
        jt.addData("LL Result", llResult == null ? "null" : "non-null");
        jt.addData("LL Valid", limelightValid);

        if (limelightPose != null) {
            jt.addData("LL Pose X", limelightPose.getX());
            jt.addData("LL Pose Y", limelightPose.getY());
            jt.addData("LL Pose H deg", Math.toDegrees(limelightPose.getHeading()));
        }

        if (lastLimelightPose != null) {
            jt.addData("LL Last X", lastLimelightPose.getX());
            jt.addData("LL Last Y", lastLimelightPose.getY());
            jt.addData("LL Last H deg", Math.toDegrees(lastLimelightPose.getHeading()));
        }

        jt.addData("B reset pose from LL", "cooldown=" + tagCooldown);
        jt.update();
        telemetryManager.update();
    }

    private Pose convertLimelightResultToPedroPose(LLResult result) {
        if (result == null || !result.isValid()) return null;

        Pose3D robotPos = result.getBotpose();
        if (robotPos == null) return null;

        double yawDeg = robotPos.getOrientation().getYaw(AngleUnit.DEGREES) + 90.0 +180;
        yawDeg = (yawDeg % 360.0 + 360.0) % 360.0;

        double xIn = (robotPos.getPosition().y * INCHES_PER_METER) + FIELD_OFFSET_IN;
        double yIn = (-robotPos.getPosition().x * INCHES_PER_METER) + FIELD_OFFSET_IN;
        double headingRad = Math.toRadians(yawDeg);

        return new Pose(xIn, yIn, headingRad);
    }

    public void stop() {
        if (USE_LIMELIGHT && limelight != null) {
            try {
                limelight.stop();
            } catch (Exception ignored) {
            }
        }
    }

    private static double wrapRad(double a) {
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        while (a > Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    private static double angleDiffRad(double a, double b) {
        return Math.abs(wrapRad(a - b));
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        if (headingLockController != null) {
            if (alliance == Alliance.BLUE) headingLockController.setGoalBlue();
            else headingLockController.setGoalRed();
        }
    }

    public double getDistanceToGoal() {
        double gx = headingLockController.getGoalX();
        double gy = headingLockController.getGoalY();
        double dx = gx - follower.getPose().getX();
        double dy = gy - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }

    private AllianceSelector.Alliance toSelectorAlliance() {
        return (alliance == Alliance.BLUE)
                ? AllianceSelector.Alliance.BLUE
                : AllianceSelector.Alliance.RED;
    }

    private Pose getOptionsResetPose() {
        AllianceSelector.Alliance a = toSelectorAlliance();

        double x = AllianceSelector.Field.resetX(a);
        double y = AllianceSelector.Field.resetY(a);

        double headingRad = Math.toRadians(270);

        return new Pose(x, y, headingRad);
    }

    private Pose getAutoEndPose() {
        AllianceSelector.Alliance a = toSelectorAlliance();

        double x = AllianceSelector.Field.EndAutoX(a);
        double y = AllianceSelector.Field.EndAutoY(a);

        double headingRad = Math.toRadians((AllianceSelector.Field.EndAutoH(a)));

        return new Pose(x, y, headingRad);
    }
}