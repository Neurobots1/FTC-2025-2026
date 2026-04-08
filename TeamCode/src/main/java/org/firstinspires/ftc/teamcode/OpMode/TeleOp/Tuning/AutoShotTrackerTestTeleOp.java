package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.PedroConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterHardwareConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TeleopConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.AutoShotBurstTracker;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShotFeedCadenceController;

@TeleOp(name = "TEST_AUTO_SHOT_TRACKER", group = "Tuning")
public class AutoShotTrackerTestTeleOp extends OpMode {

    private final ShooterConstants config = new ShooterConstants();
    private final ShotFeedCadenceController shotFeedController = new ShotFeedCadenceController();
    private final AutoShotBurstTracker burstTracker = new AutoShotBurstTracker();
    private final ElapsedTime adjustTimer = new ElapsedTime();

    private Follower follower;
    private PrecisionShooterSubsystem shooter;
    private IntakeMotor intake;

    private boolean prevY;
    private boolean prevX;
    private boolean prevA;
    private double goalX;
    private double goalY;
    private double aimGoalX;
    private double aimGoalY;
    private double rpmTrim;

    @Override
    public void init() {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(TeleopConstants.DRIVER_START_POSE);
        follower.setPose(TeleopConstants.DRIVER_START_POSE);
        follower.update();

        Indexer_Base indexerBase = new Indexer_Base(hardwareMap);
        indexerBase.StartIndexPose();
        intake = indexerBase.intkM;

        shooter = PrecisionShooterSubsystem.create(hardwareMap, follower, config);
        shooter.setReadyRequiresOnlyFlywheel(true);
        shooter.setManualHoodAngleOverrideDegrees(ShooterHardwareConstants.hoodMaxAngleDeg);
        goalX = config.blueGoalXInches;
        goalY = config.blueGoalYInches;
        aimGoalX = config.blueHeadingAimXInches;
        aimGoalY = config.blueHeadingAimYInches;
        adjustTimer.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(TeleopConstants.DRIVER_START_POSE);
        follower.setPose(TeleopConstants.DRIVER_START_POSE);
        shooter.start();
        shooter.setReadyRequiresOnlyFlywheel(true);
        shooter.setManualHoodAngleOverrideDegrees(ShooterHardwareConstants.hoodMaxAngleDeg);
        shotFeedController.setArmed(false);
        burstTracker.stop();
    }

    @Override
    public void loop() {
        handleButtons();
        updateRpmTrim();

        shooter.setAlliance(PrecisionShooterSubsystem.Alliance.BLUE);
        shooter.setGoalPosition(goalX, goalY);
        shooter.setAimPosition(aimGoalX, aimGoalY);
        shooter.setManualHoodAngleOverrideDegrees(ShooterHardwareConstants.hoodMaxAngleDeg);
        shooter.setRpmTrim(rpmTrim);
        shooter.setSpinEnabled(TeleopConstants.ALWAYS_SPIN_FLYWHEEL || shotFeedController.isArmed());
        shooter.setAutoAimEnabled(true);
        shooter.requestFire(shotFeedController.isArmed());
        shooter.update();

        PrecisionShooterSubsystem.TelemetrySnapshot snapshot = shooter.snapshot();
        shotFeedController.update(shooter.isFeedGateOpen(), snapshot);
        burstTracker.update(snapshot, shotFeedController.isBlockerSettledOpen());

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

        updateIntake();

        telemetry.addData("Target RPM", "%.0f", snapshot.targetRpm);
        telemetry.addData("Actual RPM", "%.0f", snapshot.actualRpm);
        telemetry.addData("Balls Shot", burstTracker.getDetectedShots());
        telemetry.addData("Shoot", shotFeedController.isArmed() ? "ON" : "OFF");
        telemetry.addData("Ready", snapshot.ready);
        telemetry.addLine("Y = shoot, A = reset count");
        telemetry.addLine("Dpad up/down = RPM, X = zero trim");
        telemetry.addLine("LB = intake, LT = reverse");
        telemetry.update();
    }

    @Override
    public void stop() {
        shotFeedController.setArmed(false);
        burstTracker.stop();
        if (intake != null) {
            intake.stop();
        }
    }

    private void handleButtons() {
        if (gamepad1.y && !prevY) {
            boolean arming = !shotFeedController.isArmed();
            shotFeedController.setArmed(arming);
            if (arming) {
                burstTracker.start(ShooterConstants.autoShotBurstExpectedCount);
            } else {
                burstTracker.stop();
            }
        }

        if (gamepad1.a && !prevA) {
            burstTracker.start(ShooterConstants.autoShotBurstExpectedCount);
        }

        if (gamepad1.x && !prevX) {
            rpmTrim = 0.0;
        }

        prevY = gamepad1.y;
        prevA = gamepad1.a;
        prevX = gamepad1.x;
    }

    private void updateRpmTrim() {
        if (adjustTimer.milliseconds() < 120.0) {
            return;
        }

        if (gamepad1.dpad_up) {
            rpmTrim += 25.0;
            adjustTimer.reset();
        } else if (gamepad1.dpad_down) {
            rpmTrim -= 25.0;
            adjustTimer.reset();
        }
    }

    private void updateIntake() {
        if (intake == null) {
            return;
        }

        if (gamepad1.left_trigger > 0.2) {
            intake.outtake();
        } else if (gamepad1.left_bumper) {
            intake.intake();
        } else if (shotFeedController.shouldForceFeedIntake()) {
            intake.slowIntake();
        } else {
            intake.stop();
        }
    }

}
