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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.Relocalisation;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;

@Configurable
//@TeleOp
public class Teleop_Blue_12_01_26 extends OpMode {


    public static double rawPower = -1;
    public static boolean rawPowerMode = false;
    public static boolean usePIDF = true;
    public static boolean shooterEnabled = false;
    public static double targetTicksPerSecond = 0;
    public static double currentVelocity = 0;
    public ConvertToPedroPose convertToPedroPose;
    public static double testPower = 1.0;
    private JoinedTelemetry jt;
    private Relocalisation relocalisation;
    private AprilTagPipeline aprilTagPipeline;
    private Follower follower;
    private DcMotorEx intake;
    private final Pose startingPose = new Pose(72,72,Math.toRadians(90));
    private static final double GOAL_X = 12;
    private static final double GOAL_Y = 132;
    private final Pose goalPose = new Pose(12, 132, 0.0);
    private Teleop_Blue_12_01_26.ShooterTicksLUT shooterLUT = new Teleop_Blue_12_01_26.ShooterTicksLUT();

    private final InterpLUT lut = new InterpLUT();

    private LauncherSubsystem launcher;
    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;
    private Servo Blocker;
    private VoltageSensor voltageSensor;
    private TelemetryManager telemetryManager;
    private IntakeMotor intkM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private Robot init;
    private Teleopblue1214_N_debug currentPosition;




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
        //init = new Robot(hardwareMap);
        convertToPedroPose = new ConvertToPedroPose();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcher = new LauncherSubsystem(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        launcher.init();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        aprilTagPipeline = new AprilTagPipeline(hardwareMap);
        aprilTagPipeline.startCamera();
        currentPosition = new Teleopblue1214_N_debug();
        Blocker = hardwareMap.get(Servo.class, "Blocker");
        Blocker.setPosition(1);

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(startingPose);

    }

    @Override
    public void loop(){
        follower.update();




        if (!slowMode)follower.setTeleOpDrive(
                - gamepad1.left_stick_y,
                - gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false, 3.142 // Doit etre a 0 pour rouge , mais pour bleu c'est 3.142 radian ( 180 degree)


        );

        else follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier
        );

        if (gamepad1.right_bumper) intkM.intake();
        else if (gamepad1.left_bumper) intkM.outtake();
        else intkM.stop();



        if (gamepad1.dpad_up) {
            rawPowerMode = false;
            usePIDF = true;
        }
        /*if (gamepad1.dpad_down) {
            usePIDF = false;
            rawPowerMode = true;
        } */
        if (gamepad1.a) shooterEnabled = true;
        if (gamepad1.b) shooterEnabled = false;


        if (rawPowerMode) {
            flywheelMotorOne.setPower(rawPower);
            flywheelMotorTwo.setPower(rawPower);
        } else {
            if (shooterEnabled) {
                //targetTicksPerSecond = shooterLUT.getTicksForDistance(distanceToGoal());
                launcher.setFlywheelTicks(targetTicksPerSecond);
            } else {
                launcher.setFlywheelTicks(0);
            }
            launcher.update();
        }




        double currentVelocity = flywheelMotorOne.getVelocity();


        telemetryManager.debug("shooterEnabled", shooterEnabled);
        telemetryManager.debug("usePIDF", usePIDF);
        telemetryManager.debug("targetTicksPerSecond", targetTicksPerSecond);
        telemetryManager.debug("testPower", testPower);
        telemetryManager.debug("P", LauncherSubsystem.P);
        telemetryManager.debug("I", LauncherSubsystem.I);
        telemetryManager.debug("D", LauncherSubsystem.D);
        telemetryManager.debug("F", LauncherSubsystem.F);
        telemetryManager.debug("NOMINAL_VOLTAGE", LauncherSubsystem.NOMINAL_VOLTAGE);
        telemetryManager.debug("pose2D", follower.getPose());
        jt.addData("targetTicksPerSecond", "%.0f", targetTicksPerSecond);
        jt.addData("currentVelocity", "%.0f", currentVelocity);
        jt.addData("positon", follower.getPose());
        jt.addData("Power", intake.getPower());
        jt.addData("Velocity", follower.getVelocity().getMagnitude());
        jt.update();
        telemetryManager.update();

    }


    private static class ShooterTicksLUT {

        // Distances in SAME UNITS as follower.getPose()
        private final double[] distances = {
                40, 50
        };

        // Matching flywheel target velocities in ticks/second
        private final double[] ticks = {
                600, 700
        };

        private final Teleop_Blue_12_01_26.ShooterTicksLUT.CubicHermiteInterpolator hermite;

        ShooterTicksLUT() {
            hermite = new Teleop_Blue_12_01_26.ShooterTicksLUT.CubicHermiteInterpolator(distances, ticks);
        }

        public double getTicksForDistance(double d) {
            return hermite.interpolate(d);
        }

        // Monotonic Cubic Hermite Interpolator
        private static class CubicHermiteInterpolator {
            private final double[] x;
            private final double[] y;
            private final double[] m; // slopes

            CubicHermiteInterpolator(double[] x, double[] y) {
                if (x.length != y.length) {
                    throw new IllegalArgumentException("Arrays must be same length");
                }
                this.x = x;
                this.y = y;
                this.m = computeSlopes(x, y);
            }

            private double[] computeSlopes(double[] x, double[] y) {
                int n = x.length;
                double[] m = new double[n];
                double[] d = new double[n - 1];

                // secant slopes between points
                for (int i = 0; i < n - 1; i++) {
                    d[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
                }

                // initial tangents
                m[0] = d[0];
                for (int i = 1; i < n - 1; i++) {
                    if (d[i - 1] * d[i] <= 0) {
                        m[i] = 0.0; // slope sign change → flatten
                    } else {
                        m[i] = 0.5 * (d[i - 1] + d[i]);
                    }
                }
                m[n - 1] = d[n - 2];

                // enforce monotonicity more precise
                for (int i = 0; i < n - 1; i++) {
                    if (Math.abs(d[i]) < 1e-9) {
                        m[i] = 0.0;
                        m[i + 1] = 0.0;
                    } else {
                        double a = m[i] / d[i];
                        double b = m[i + 1] / d[i];
                        double r = a * a + b * b;
                        if (r > 9.0) {
                            double t = 3.0 / Math.sqrt(r);
                            m[i] = t * a * d[i];
                            m[i + 1] = t * b * d[i];
                        }
                    }
                }
                return m;
            }

            public double interpolate(double xVal) {
                int n = x.length;

                // clamp out of range
                if (xVal <= x[0]) return y[0];
                if (xVal >= x[n - 1]) return y[n - 1];

                // find interval
                int i = 0;
                while (i < n - 1 && xVal > x[i + 1]) i++;

                double h = x[i + 1] - x[i];
                double t = (xVal - x[i]) / h;

                double y0 = y[i];
                double y1 = y[i + 1];
                double m0 = m[i] * h;
                double m1 = m[i + 1] * h;

                double t2 = t * t;
                double t3 = t2 * t;

                double h00 = 2 * t3 - 3 * t2 + 1;
                double h10 = t3 - 2 * t2 + t;
                double h01 = -2 * t3 + 3 * t2;
                double h11 = t3 - t2;

                return h00 * y0 + h10 * m0 + h01 * y1 + h11 * m1;
            }
        }
    }

  /*  public double distanceToGoal() {
        double goalX = 12;
        double goalY = 136;
        double dx = goalX - follower.getPose().getX();
        double dy = goalY - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }

        public static boolean isInShootingZone(double x, double y) {
        return isInBackZone(x, y) || isInFrontZone(x, y);
    }

        public static boolean isInBackZone(double x, double y) {
        return y <= x - 48 && y <= -x + 96;
    }

        public static boolean isInFrontZone(double x, double y) {
        return y >= -x + 144 && y >= x;
    } */

    private double calculateTargetHeading() {
        Pose currentPose = follower.getPose();
        double deltaX = GOAL_X - currentPose.getX();
        double deltaY = GOAL_Y - currentPose.getY();
        return Math.atan2(deltaY, deltaX);
    }

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
        double kP = 1.0; // Adjust this value based on testing

        // Calculate heading correction power
        double headingPower = headingError * kP;

        // Clamp the output to [-1, 1]
        headingPower = Math.max(-1.0, Math.min(1.0, headingPower));

        return headingPower;
    }

}