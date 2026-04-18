package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Constants.LUTConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterHardwareConstants;
import org.firstinspires.ftc.teamcode.Constants.TeleopConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.Constants.PedroConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Locale;

@TeleOp(name = "TUNE_SHOT_TABLE", group = "Tuning")
public class PrecisionShooterTrainingOpMode extends OpMode {

    private static final String LOG_TAG = "PrecisionShooterTrainer";
    private static final int EXPORTED_SHOT_ROWS = 12;

    private final ShooterConstants config = new ShooterConstants();
    private final ArrayList<PrecisionShotTable.Entry> samples = new ArrayList<>();
    private final ElapsedTime adjustTimer = new ElapsedTime();

    private Follower follower;
    private FlywheelVelocityController flywheel;
    private ServoHoodController hood;
    private IntakeMotor intake;

    private boolean spinEnabled;
    private boolean prevA;
    private boolean prevBButton;
    private boolean prevRightStickButton;
    private boolean prevX;
    private boolean prevY;
    private boolean prevOptions;
    private double manualRpm = 3000.0;
    private double manualHoodDeg = 40.0;
    private PrecisionShooterSubsystem.Alliance alliance = PrecisionShooterSubsystem.Alliance.BLUE;

    @Override
    public void init() {
        samples.clear();
        telemetry.log().clear();
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(TeleopConstants.DRIVER_START_POSE);
        follower.setPose(TeleopConstants.DRIVER_START_POSE);
        follower.update();

        DcMotorEx left = hardwareMap.get(DcMotorEx.class, ShooterHardwareConstants.leftFlywheelName);
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, ShooterHardwareConstants.rightFlywheelName);
        Servo hoodServo = hardwareMap.get(Servo.class, ShooterHardwareConstants.hoodServoName);
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        Indexer_Base indexerBase = new Indexer_Base(hardwareMap);
        indexerBase.StartIndexPose();

        flywheel = new FlywheelVelocityController(left, right, voltageSensor, config);
        hood = new ServoHoodController(hoodServo, config);
        intake = indexerBase.intkM;
        adjustTimer.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(TeleopConstants.DRIVER_START_POSE);
        follower.setPose(TeleopConstants.DRIVER_START_POSE);
        telemetry.log().clear();
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

        if (adjustTimer.milliseconds() >= 120.0) {
            if (gamepad1.dpad_up) {
                manualRpm += 25.0;
                adjustTimer.reset();
            } else if (gamepad1.dpad_down) {
                manualRpm -= 25.0;
                adjustTimer.reset();
            } else if (gamepad1.dpad_right) {
                manualHoodDeg += 0.25;
                adjustTimer.reset();
            } else if (gamepad1.dpad_left) {
                manualHoodDeg -= 0.25;
                adjustTimer.reset();
            }
        }

        manualRpm = ShooterMath.clamp(manualRpm, 0.0, 5000.0);
        manualHoodDeg = ShooterMath.clamp(
                manualHoodDeg,
                ShooterHardwareConstants.hoodMinAngleDeg,
                ShooterHardwareConstants.hoodMaxAngleDeg
        );

        if (gamepad1.right_stick_button && !prevRightStickButton) {
            spinEnabled = !spinEnabled;
        }

        if (gamepad1.y && !prevY) {
            alliance = alliance == PrecisionShooterSubsystem.Alliance.BLUE
                    ? PrecisionShooterSubsystem.Alliance.RED
                    : PrecisionShooterSubsystem.Alliance.BLUE;
        }

        if (gamepad1.options && !prevOptions) {
            follower.setStartingPose(TeleopConstants.DRIVER_START_POSE);
            follower.setPose(TeleopConstants.DRIVER_START_POSE);
        }

        hood.setAngleDegrees(manualHoodDeg);
        hood.update();
        flywheel.setTargetRpm(spinEnabled ? manualRpm : 0.0);
        flywheel.update(config.flywheelIntegralLimit);
        updateIntake();

        Pose pose = follower.getPose();
        double goalX = alliance == PrecisionShooterSubsystem.Alliance.BLUE ? config.blueGoalXInches : config.redGoalXInches;
        double goalY = alliance == PrecisionShooterSubsystem.Alliance.BLUE ? config.blueGoalYInches : config.redGoalYInches;
        double distance = Math.hypot(goalX - pose.getX(), goalY - pose.getY());

        if (gamepad1.a && !prevA) {
            samples.add(new PrecisionShotTable.Entry(distance, manualRpm, manualHoodDeg));
        }

        if (gamepad1.x && !prevX && !samples.isEmpty()) {
            samples.remove(samples.size() - 1);
        }

        if (gamepad1.b && !prevBButton) {
            emitTable();
        }

        prevA = gamepad1.a;
        prevBButton = gamepad1.b;
        prevRightStickButton = gamepad1.right_stick_button;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
        prevOptions = gamepad1.options;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose", pose);
        telemetry.addData("Goal", "(%.1f, %.1f)", goalX, goalY);
        telemetry.addData("Distance To Goal", "%.2f", distance);
        telemetry.addData("Spin", spinEnabled);
        telemetry.addData("Manual RPM", "%.1f", manualRpm);
        telemetry.addData("Actual RPM", "%.1f", flywheel.getMeasuredRpm());
        telemetry.addData("Hood Deg", "%.2f", manualHoodDeg);
        telemetry.addData("Samples", samples.size());
        telemetry.addLine("RS click = spin toggle, RB = intake, LB = outtake");
        telemetry.addLine("A = record, X = delete last, B = print Panels rows, Y = swap alliance");
        telemetry.addLine("Options = reset pose to (72,72,90)");
        if (!samples.isEmpty()) {
            PrecisionShotTable.Entry last = samples.get(samples.size() - 1);
            telemetry.addData("Last Sample", Arrays.asList(last.distanceInches, last.targetRpm, last.hoodAngleDeg));
        }
        telemetry.update();
    }

    private void emitTable() {
        telemetry.log().clear();
        if (samples.isEmpty()) {
            telemetry.log().add("No samples recorded yet.");
            return;
        }
        ArrayList<PrecisionShotTable.Entry> sorted = new ArrayList<>(samples);
        sorted.sort(Comparator.comparingDouble(e -> e.distanceInches));
        PrecisionShotTable table = new PrecisionShotTable(sorted);
        String javaOutput = table.toJavaInitializer("COMP_TABLE");
        String panelsOutput = toPanelsAssignments(sorted);
        RobotLog.ii(LOG_TAG, javaOutput);
        RobotLog.ii(LOG_TAG, panelsOutput);
        telemetry.log().add("Table logged to RobotLog.");
        telemetry.log().add(panelsOutput);
    }

    private String toPanelsAssignments(ArrayList<PrecisionShotTable.Entry> sorted) {
        StringBuilder builder = new StringBuilder();
        builder.append("LUTConstants Panels rows:\n");
        for (int i = 0; i < EXPORTED_SHOT_ROWS; i++) {
            PrecisionShotTable.Entry entry = i < sorted.size()
                    ? sorted.get(i)
                    : sorted.get(sorted.size() - 1);
            int row = i + 1;
            builder.append(String.format(Locale.US, "shot%dDistanceInches = %.2f%n", row, entry.distanceInches));
            builder.append(String.format(Locale.US, "shot%dTargetRpm = %.1f%n", row, entry.targetRpm));
            builder.append(String.format(Locale.US, "shot%dHoodAngleDeg = %.2f%n", row, entry.hoodAngleDeg));
        }
        return builder.toString();
    }

    @Override
    public void stop() {
        if (intake != null) {
            intake.stop();
        }
        if (flywheel != null) {
            flywheel.stop();
        }
    }

    private void updateIntake() {
        if (intake == null) {
            return;
        }

        if (gamepad1.right_bumper) {
            intake.intake();
        } else if (gamepad1.left_bumper) {
            intake.outtake();
        } else {
            intake.stop();
        }
    }
}
