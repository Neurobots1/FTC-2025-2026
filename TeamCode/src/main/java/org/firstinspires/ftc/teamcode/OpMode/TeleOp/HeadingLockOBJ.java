package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class HeadingLockOBJ extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    // Shooting mode variables
    private boolean shootingMode = false;

    // Configure your goal position here (x, y coordinates on the field)
    private static final double GOAL_X = 72.0; // Example: 72 inches
    private static final double GOAL_Y = 72.0; // Example: 72 inches

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            double headingInput;

            // Calculate heading based on mode
            if (shootingMode) {
                // Calculate heading to aim at goal
                headingInput = calculateHeadingToGoal();
            } else {
                // Normal heading control from right stick
                headingInput = -gamepad1.right_stick_x;
            }

            // Apply drive with calculated heading
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    headingInput,
                    true // Robot Centric
            );
        }

        // Toggle shooting mode with left bumper
        if (gamepad1.leftBumperWasPressed()) {
            shootingMode = !shootingMode;
        }

        // Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        // Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Telemetry
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("shootingMode", shootingMode);
        if (shootingMode) {
            telemetryM.debug("targetHeading (deg)", Math.toDegrees(calculateTargetHeading()));
        }
    }

    /**
     * Calculates the target heading to aim at the goal
     * @return target heading in radians
     */
    private double calculateTargetHeading() {
        Pose currentPose = follower.getPose();
        double deltaX = GOAL_X - currentPose.getX();
        double deltaY = GOAL_Y - currentPose.getY();
        return Math.atan2(deltaY, deltaX);
    }

    /**
     * Calculates the heading power needed to aim at the goal from the robot's current position
     * @return heading value to pass to setTeleOpDrive (rotation rate/power)
     */
    private double calculateHeadingToGoal() {
        // Get current robot pose
        Pose currentPose = follower.getPose();

        // Calculate target heading (angle to goal)
        double targetHeading = calculateTargetHeading();

        // Get current heading
        double currentHeading = currentPose.getHeading();

        // Calculate heading error
        double headingError = targetHeading - currentHeading;

        // Normalize heading error to [-PI, PI]
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        // P controller for heading correction
        // Tune this gain value to control how aggressively the robot turns
        double kP = 2.0; // Adjust this value based on testing

        // Calculate heading correction power
        double headingPower = headingError * kP;

        // Clamp the output to [-1, 1]
        headingPower = Math.max(-1.0, Math.min(1.0, headingPower));

        return headingPower;
    }
}