package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.HeadingLockController;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {

    public static boolean USE_LIMELIGHT = false;

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
    private LLResult llResult;

    private boolean visionInitialized = false;
    private double yawBiasRad = 0.0;

    private final ElapsedTime shooterReadyTimer = new ElapsedTime();
    private boolean headingLockWasEnabled = false;
    private boolean shooterReadyRumbleSent = false;

    private final Pose startingPose = new Pose(72, 72, Math.toRadians(90));


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
            limelight.pipelineSwitch(1);
            visionInitialized = false;
            yawBiasRad = 0.0;
        }
    }

    public void startTeleop() {
        follower.startTeleopDrive();
        follower.setStartingPose(startingPose);

        if (USE_LIMELIGHT && limelight != null) {
            limelight.start();
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

        if (USE_LIMELIGHT && limelight != null && !visionInitialized) {

            double yawForLLDeg = Math.toDegrees(wrapRad(currentPose.getHeading() + yawBiasRad));
            limelight.updateRobotOrientation(yawForLLDeg);

            llResult = limelight.getLatestResult();

            boolean llValid = false;
            Pose3D llPose3d = null;

            if (llResult != null && llResult.isValid()) {
                llPose3d = llResult.getBotpose_MT2();
                llValid = (llPose3d != null);
            }

            int tagCount = 0;
            if (llValid && llResult.getDetectorResults() != null) {
                tagCount = llResult.getDetectorResults().size();
            }

            jt.addData("LL enabled", true);
            jt.addData("LL valid", llValid);
            jt.addData("LL tagCount", tagCount);

            boolean stillEnough =
                    follower.getVelocity().getMagnitude() < 0.2
                            && Math.abs(follower.getAngularVelocity()) < 0.1;

            if (llValid
                    && tagCount > 0
                    && tagResetTimer.seconds() > tagCooldown
                    && stillEnough) {

                Pose visionPose = getRobotPoseFromCamera();
                if (visionPose != null) {

                    double imuHeadingRad = currentPose.getHeading();

                    double h0 = wrapRad(visionPose.getHeading());
                    double h1 = wrapRad(visionPose.getHeading() + Math.PI);

                    double chosen =
                            angleDiffRad(h0, imuHeadingRad) <= angleDiffRad(h1, imuHeadingRad)
                                    ? h0
                                    : h1;

                    yawBiasRad = wrapRad(chosen - imuHeadingRad);

                    double correctedHeading = wrapRad(imuHeadingRad + yawBiasRad);

                    follower.setPose(new Pose(
                            visionPose.getX(),
                            visionPose.getY(),
                            correctedHeading
                    ));

                    visionInitialized = true;
                    tagResetTimer.reset();
                }
            }
        } else {
            jt.addData("LL enabled", USE_LIMELIGHT);
            jt.addData("LL initDone", visionInitialized);
        }

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

        if (gamepad.dpad_down) {
            indexerBase.startOutTake();
        }

        indexerBase.OutTake();

        jt.addData("Alliance", alliance);
        jt.addData("position", follower.getPose());
        jt.addData("Velocity", follower.getVelocity().getMagnitude());
        jt.addData("distance to goal", getDistanceToGoal());
        jt.update();
        telemetryManager.update();
    }

    private static double wrapRad(double a) {
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        while (a > Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    private static double angleDiffRad(double a, double b) {
        return Math.abs(wrapRad(a - b));
    }

    private Pose getRobotPoseFromCamera() {
        if (!USE_LIMELIGHT) return null;
        if (llResult == null || !llResult.isValid()) return null;

        Pose3D llPose = llResult.getBotpose_MT2();
        if (llPose == null) return null;

        double xIn = llPose.getPosition().toUnit(DistanceUnit.INCH).x;
        double yIn = llPose.getPosition().toUnit(DistanceUnit.INCH).y;

        double camHeadingRad =
                Math.toRadians(llPose.getOrientation().getYaw(AngleUnit.DEGREES));

        Pose2D apriltagPose = new Pose2D(
                DistanceUnit.INCH,
                xIn,
                yIn,
                AngleUnit.RADIANS,
                camHeadingRad
        );

        Pose ftcStandard = PoseConverter.pose2DToPose(
                apriltagPose,
                InvertedFTCCoordinates.INSTANCE
        );

        return ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
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
}
