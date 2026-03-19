package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tuning;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.PrecisionShooterConfig;
import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TUNE_SHOOTER_SYSTEM", group = "Tuning")
public class PrecisionShooterTuningTeleOp extends OpMode {

    private final PrecisionShooterConfig config = new PrecisionShooterConfig();
    private final ElapsedTime adjustTimer = new ElapsedTime();

    private Follower follower;
    private PrecisionShooterSubsystem shooter;
    private IntakeMotor intake;
    private PrecisionShooterSubsystem.Alliance alliance = PrecisionShooterSubsystem.Alliance.BLUE;
    private boolean spinEnabled;
    private boolean autoAimEnabled = true;
    private boolean prevB;
    private boolean prevX;
    private boolean prevY;
    private double goalX;
    private double goalY;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        Indexer_Base indexerBase = new Indexer_Base(hardwareMap);
        indexerBase.StartIndexPose();
        intake = indexerBase.intkM;
        shooter = PrecisionShooterSubsystem.create(hardwareMap, follower, config);
        applyAllianceDefaults();
        adjustTimer.reset();
    }

    @Override
    public void start() {
        shooter.start();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true,
                AllianceSelector.Field.fieldCentricOffset(
                        alliance == PrecisionShooterSubsystem.Alliance.BLUE
                                ? AllianceSelector.Alliance.BLUE
                                : AllianceSelector.Alliance.RED
                )
        );

        if (gamepad1.y && !prevY) {
            spinEnabled = !spinEnabled;
        }
        if (gamepad1.x && !prevX) {
            autoAimEnabled = !autoAimEnabled;
        }
        if (gamepad1.b && !prevB) {
            alliance = alliance == PrecisionShooterSubsystem.Alliance.BLUE
                    ? PrecisionShooterSubsystem.Alliance.RED
                    : PrecisionShooterSubsystem.Alliance.BLUE;
            applyAllianceDefaults();
        }

        if (adjustTimer.milliseconds() >= 120.0) {
            if (gamepad1.dpad_up) {
                goalX += 2.0;
                adjustTimer.reset();
            } else if (gamepad1.dpad_down) {
                goalX -= 2.0;
                adjustTimer.reset();
            } else if (gamepad1.dpad_right) {
                goalY += 2.0;
                adjustTimer.reset();
            } else if (gamepad1.dpad_left) {
                goalY -= 2.0;
                adjustTimer.reset();
            }
        }

        if (gamepad1.a) {
            applyAllianceDefaults();
        }

        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;

        shooter.setAlliance(alliance);
        shooter.setGoalPosition(goalX, goalY);
        shooter.setSpinEnabled(spinEnabled);
        shooter.setAutoAimEnabled(autoAimEnabled);
        shooter.requestFire(gamepad1.right_bumper);
        shooter.update();

        PrecisionShooterSubsystem.TelemetrySnapshot snapshot = shooter.snapshot();
        updateFeedAssist(snapshot.ready);

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Goal", "(%.1f, %.1f)", goalX, goalY);
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Spin", spinEnabled);
        telemetry.addData("Auto Aim", autoAimEnabled);
        telemetry.addData("Homed", snapshot.homed);
        telemetry.addData("Shot Zone", snapshot.inShootingZone);
        telemetry.addData("Ready", snapshot.ready);
        telemetry.addData("Target RPM", "%.1f", snapshot.targetRpm);
        telemetry.addData("Actual RPM", "%.1f", snapshot.actualRpm);
        telemetry.addData("Nominal Hood", "%.2f", snapshot.nominalHoodDeg);
        telemetry.addData("Comp Hood", "%.2f", snapshot.compensatedHoodDeg);
        telemetry.addData("Turret", "%.2f / %.2f", snapshot.turretAngleDeg, snapshot.turretTargetDeg);
        telemetry.addData("Predicted Range", "%.2f", snapshot.predictedRangeInches);
        telemetry.addData("TOF", "%.3f", snapshot.timeOfFlightSeconds);
        telemetry.addData("Status", snapshot.status);
        telemetry.addData("Feed Assist", gamepad1.left_bumper ? "Manual Intake" : (gamepad1.left_trigger > 0.2 ? "Reverse" : (gamepad1.right_bumper && snapshot.ready ? "Auto Feed" : "Off")));
        telemetry.addLine("Y spin, X auto-aim, RB fire, LB intake, LT reverse");
        telemetry.addLine("B alliance, A reset goal");
        telemetry.addLine("Dpad adjusts goal X/Y by 2 inches");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (intake != null) {
            intake.stop();
        }
    }

    private void applyAllianceDefaults() {
        goalX = alliance == PrecisionShooterSubsystem.Alliance.BLUE
                ? config.blueGoalXInches
                : config.redGoalXInches;
        goalY = alliance == PrecisionShooterSubsystem.Alliance.BLUE
                ? config.blueGoalYInches
                : config.redGoalYInches;
    }

    private void updateFeedAssist(boolean shooterReady) {
        if (intake == null) {
            return;
        }

        if (gamepad1.left_trigger > 0.2) {
            intake.outtake();
        } else if (gamepad1.left_bumper) {
            intake.intake();
        } else if (gamepad1.right_bumper && shooterReady) {
            intake.slowIntake();
        } else {
            intake.stop();
        }
    }
}
