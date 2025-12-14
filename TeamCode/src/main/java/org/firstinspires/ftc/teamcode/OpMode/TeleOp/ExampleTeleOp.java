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
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class ExampleTeleOp extends OpMode {

    // ---------------------------------------
    // Path following objects
    // ---------------------------------------
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    // ---------------------------------------
    // Slow mode
    // ---------------------------------------
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // ---------------------------------------
    // Intake
    // ---------------------------------------
    private DcMotorEx intake;

    // ---------------------------------------
    // Shooter (Two Motors, One Velocity PIDF)
    // ---------------------------------------
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // Editable PIDF values (can tune live if @Configurable dashboard is used)
    public static double kP = 0.0007;
    public static double kI = 0.0;
    public static double kD = 0.0001;
    public static double kF = 0.00018;

    // Shooter target RPM
    public static double targetRpm = 0;

    // Internal PID variables
    private double shooterIntegral = 0;
    private double lastError = 0;
    private long lastTime;

    @Override
    public void init() {

        // ------------------------------
        // Path Following Engine
        // ------------------------------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        // ------------------------------
        // Hardware
        // ------------------------------
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "ShooterA");
        shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterB");

        shooterLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastTime = System.nanoTime();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        // =============================================================
        // DRIVER CONTROLLED DRIVING
        // =============================================================
        if (!automatedDrive) {

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if (slowMode)
                follower.setTeleOpDrive(y * slowModeMultiplier, x * slowModeMultiplier, turn * slowModeMultiplier, true);
            else
                follower.setTeleOpDrive(y, x, turn, true);
        }

        // =============================================================
        // INTAKE CONTROL
        // =============================================================

        if (gamepad1.right_bumper) {
            intake.setPower(1);            // Forward
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1);           // Reverse
        } else {
            intake.setPower(0);            // Off
        }

        // =============================================================
        // SHOOTER CONTROL (Velocity PIDF)
        // Gamepad1:
        //    Dpad Up:   Increase RPM
        //    Dpad Down: Decrease RPM
        // =============================================================

        if (gamepad1.dpad_up)   targetRpm += 50;
        if (gamepad1.dpad_down) targetRpm -= 50;
        if (targetRpm < 0) targetRpm = 0;

        runShooterPIDF();

        // =============================================================
        // AUTOMATED PATHFOLLOWS
        // =============================================================
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // =============================================================
        // Slowmode toggle
        // =============================================================
        if (gamepad1.rightBumperWasPressed())
            slowMode = !slowMode;

        // =============================================================
        // TELEMETRY
        // =============================================================
        telemetryM.debug("Pose", follower.getPose());
        telemetryM.debug("Velocity", follower.getVelocity());
        telemetryM.debug("Shooter Target RPM", targetRpm);
        telemetryM.debug("Shooter Current RPM", getShooterRpm());
        telemetryM.debug("Automated Drive", automatedDrive);
    }


    // =========================================================================
    // SHOOTER PIDF CONTROL
    // =========================================================================

    private double getShooterRpm() {
        // read velocity of ONE shooter motor (ticks per second)
        double tps = shooterLeft.getVelocity();

        // convert ticks/sec → RPM
        return (tps / 28.0) * 60.0; // (use your motor's ticks per rev!)
    }

    private void runShooterPIDF() {
        double currentRpm = getShooterRpm();
        double error = targetRpm - currentRpm;

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        shooterIntegral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        // PIDF output
        double output =
                (kP * error) +
                        (kI * shooterIntegral) +
                        (kD * derivative) +
                        (kF * (targetRpm / 6000.0)); // feedforward normalized

        output = Math.max(0, Math.min(output, 1)); // safety clamp

        shooterLeft.setPower(-output);
        shooterRight.setPower(output);
    }
}

