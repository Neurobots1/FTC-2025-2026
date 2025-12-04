package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.SubSystem.Shoot;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.ShooterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class TeleopFinalBlue extends OpMode {

    private Follower follower;
    private IntakeMotor intkM;
    private Shoot shootM;
    private Robot init;

    private boolean shootingMode = false;
    private boolean automatedHeading;
    private TelemetryManager telemetryM;

    private DcMotorEx intake;
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // ================== SHOOTER CONSTANTS ===================
    public static final double TICKS_PER_REV = 112.0; // ✔ correct for GoBILDA 5203-2402-0001 (6000rpm 1:1)
    public static final double MAX_RPM = 6000.0;

    // PIDF GAINS
    public static double kP = 0.10;
    public static double kI = 0;
    public static double kD = 0.00004;
    public static double kF = 1.0 / MAX_RPM;       // ✔ proper feedforward

    public static double targetRpm = 1000;

    private double shooterIntegral = 0;
    private double lastError = 0;
    private long lastTime;

    private final Pose startingPose = new Pose(63, 136, Math.toRadians(-90));
    private PanelsTelemetry panelsTelemetry;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "ShooterA");
        shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterB");

        shooterLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // ✔ Correct shooter directions (stop using negative power)
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intkM = new IntakeMotor(hardwareMap);
        init = new Robot(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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

        // ---------------- DRIVE CONTROL -----------------
        double headingInput =
                shootingMode ? calculateHeadingToGoal() : -gamepad1.right_stick_x;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                headingInput,
                false
        );

        if (gamepad1.leftBumperWasPressed()) {
            shootingMode = !shootingMode;
        }

        if (gamepad1.right_bumper) intake.setPower(1);
        else if (gamepad1.left_bumper) intake.setPower(-1);
        else intake.setPower(0);

        // ------------ SHOOTER PIDF --------------
        runShooterPIDF();

        // ------------ TELEMETRY ---------------
        telemetryM.debug("Shooter RPM", getShooterRpm());
        telemetryM.debug("Target RPM", targetRpm);
        telemetryM.debug("Output Left", shooterLeft.getPower());
        telemetryM.debug("Output Right", shooterRight.getPower());
        telemetryM.debug("Shooting Mode", shootingMode);
    }

    // ================== SHOOTER MATH ===================

    private double getShooterRpm() {
        double tps = shooterLeft.getVelocity();       // ticks per second
        return (tps / TICKS_PER_REV) * 60.0;          // ✔ correct conversion to RPM
    }

    private void runShooterPIDF() {
        double currentRpm = getShooterRpm();
        double error = targetRpm - currentRpm;

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        shooterIntegral += (error * dt);
        double derivative = (error - lastError) / dt;
        lastError = error;

        // ✔ Correct feedforward: rpm/maxRPM
        double ff = targetRpm * kF;

        double output =
                (kP * error) +
                        (kI * shooterIntegral) +
                        (kD * derivative) +
                        ff;

        output = Math.max(0, Math.min(output, 1));   // clamp 0–1

        shooterLeft.setPower(output);
        shooterRight.setPower(output);
    }

    // ================== HEADING TO GOAL ===================
    private double calculateTargetHeading() {
        Pose p = follower.getPose();
        double dx = 12 - p.getX();
        double dy = 132 - p.getY();
        return Math.atan2(dy, dx);
    }

    private double calculateHeadingToGoal() {
        Pose p = follower.getPose();
        double target = calculateTargetHeading();
        double error = target - p.getHeading();

        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        double kP = 1.0;
        double power = error * kP;

        return Math.max(-1, Math.min(1, power));
    }
}
