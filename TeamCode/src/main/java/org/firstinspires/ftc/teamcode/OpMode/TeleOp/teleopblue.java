package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
//@TeleOp(name = "teleopblue", group = "Tuning")
public class teleopblue extends OpMode {

    public static boolean usePIDF = true;
    public static boolean shooterEnabled = false;
    public static double targetTicksPerSecond = 0;
    public static double testPower = 1.0;
    private JoinedTelemetry jt;
    private Robot robot;
    private Follower follower;
    private DcMotorEx intake;
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(90));
    private static final double GOAL_X = 12; // Example: 72 inches
    private static final double GOAL_Y = 132; // Example: 72 inches
    private final Pose goalPose = new Pose(12, 132, 0.0);
    private ShooterTicksLUT shooterLUT = new ShooterTicksLUT();

    private final InterpLUT lut = new InterpLUT();





    private LauncherSubsystem launcher;
    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;
    private VoltageSensor voltageSensor;
    private TelemetryManager telemetryManager;
    private IntakeMotor intkM;
    private Robot init;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        jt = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        intkM = new IntakeMotor(hardwareMap);
        robot = new Robot();
        robot.init(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcher = new LauncherSubsystem(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        launcher.init();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        double headingInput =
                shooterEnabled ? calculateHeadingToGoal() : -gamepad1.right_stick_x;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                headingInput,
                false
        );

        if (gamepad1.right_bumper) intake.setPower(1);
        else if (gamepad1.left_bumper) intake.setPower(-1);
        else intake.setPower(0);

        if (gamepad1.a) shooterEnabled = true;
        if (gamepad1.b) shooterEnabled = false;

        Pose robotPose = follower.getPose();

        if (shooterEnabled) {
           // targetTicksPerSecond = shooterLUT.getTicksForDistance(distanceToGoal());

            launcher.setFlywheelTicks(targetTicksPerSecond);
        } else {
            launcher.setFlywheelTicks(0);
        }
        launcher.update();


        double currentVelocity = flywheelMotorOne.getVelocity();

        telemetryManager.debug("shooterEnabled", shooterEnabled);
        telemetryManager.debug("usePIDF", usePIDF);
        telemetryManager.debug("targetTicksPerSecond", targetTicksPerSecond);
        telemetryManager.debug("testPower", testPower);
        telemetryManager.debug("currentVelocity", currentVelocity);
        telemetryManager.debug("P", LauncherSubsystem.P);
        telemetryManager.debug("I", LauncherSubsystem.I);
        telemetryManager.debug("D", LauncherSubsystem.D);
        telemetryManager.debug("F", LauncherSubsystem.F);
        telemetryManager.debug("NOMINAL_VOLTAGE", LauncherSubsystem.NOMINAL_VOLTAGE);
        telemetryManager.debug("pose2D", follower.getPose());
        telemetryManager.debug("distance goal", distanceToGoal());
        jt.addData("targetTicksPerSecond", "%.0f", targetTicksPerSecond);
        jt.addData("currentVelocity", "%.0f", currentVelocity);
        telemetryManager.update();
    }
    private double calculateTargetHeading() {
        com.pedropathing.geometry.Pose currentPose = follower.getPose();
        double deltaX = GOAL_X - currentPose.getX();
        double deltaY = GOAL_Y - currentPose.getY();
        return Math.atan2(deltaY, deltaX);
    }

    private double calculateHeadingToGoal() {
        // Get current robot pose
        com.pedropathing.geometry.Pose currentPose = follower.getPose();

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
        double kP = 1.0; // Adjust this value based on testing

        // Calculate heading correction power
        double headingPower = headingError * kP;

        // Clamp the output to [-1, 1]
        headingPower = Math.max(-1.0, Math.min(1.0, headingPower));

        return headingPower;
    }
    public static class ShooterTicksLUT {
        // Distances in SAME UNITS as follower.getPose() (your Pedro field units)
        private final double[] distances = {
               44, 51, 63, 71, 91, 101   // <-- put your tested distances here
        };

        // Matching flywheel target velocities in ticks/second
        private final double[] ticks = {
               600, 670, 720, 780, 820, 850   // <-- your tuned values
        };

        public double getTicksForDistance(double d) {

            // Clamp out-of-range
            if (d <= distances[0]) return ticks[0];
            if (d >= distances[distances.length - 1]) return ticks[ticks.length - 1];

            // Linear interpolation between the two nearest points
            for (int i = 0; i < distances.length - 1; i++) {
                double d0 = distances[i];
                double d1 = distances[i + 1];

                if (d >= d0 && d <= d1) {
                    double t = (d - d0) / (d1 - d0);   // 0..1
                    return ticks[i] + t * (ticks[i + 1] - ticks[i]);
                }
            }

            // Should never hit this
            return ticks[ticks.length - 1];
        }
    }



    public double distanceToGoal() {
        // Goal coordinates (Pedro uses cm)
        double goalX = 6;
        double goalY = 136;
        double dx = goalX - follower.getPose().getX();
        double dy = goalY - follower.getPose().getY();

        return Math.hypot(dx, dy);
    }

}

