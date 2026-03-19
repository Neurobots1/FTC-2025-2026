package org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Constants.PrecisionShooterConfig;
import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Locale;

@TeleOp(name = "TUNE_SHOT_TABLE", group = "Tuning")
public class PrecisionShooterTrainingOpMode extends OpMode {

    private static final String LOG_TAG = "PrecisionShooterTrainer";

    private final PrecisionShooterConfig config = new PrecisionShooterConfig();
    private final ArrayList<PrecisionShotTable.Entry> samples = new ArrayList<>();
    private final ElapsedTime adjustTimer = new ElapsedTime();

    private Follower follower;
    private FlywheelVelocityController flywheel;
    private ServoHoodController hood;

    private boolean spinEnabled;
    private boolean prevA;
    private boolean prevBButton;
    private boolean prevRightBumper;
    private boolean prevX;
    private boolean prevY;
    private double manualRpm = 3000.0;
    private double manualHoodDeg = 40.0;
    private PrecisionShooterSubsystem.Alliance alliance = PrecisionShooterSubsystem.Alliance.BLUE;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        DcMotorEx left = hardwareMap.get(DcMotorEx.class, config.leftFlywheelName);
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, config.rightFlywheelName);
        Servo hoodServo = hardwareMap.get(Servo.class, config.hoodServoName);
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheel = new FlywheelVelocityController(left, right, voltageSensor, config);
        hood = new ServoHoodController(hoodServo, config);
        adjustTimer.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
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
        manualHoodDeg = ShooterMath.clamp(manualHoodDeg, config.hoodMinAngleDeg, config.hoodMaxAngleDeg);

        if (gamepad1.right_bumper && !prevRightBumper) {
            spinEnabled = !spinEnabled;
        }

        if (gamepad1.y && !prevY) {
            alliance = alliance == PrecisionShooterSubsystem.Alliance.BLUE
                    ? PrecisionShooterSubsystem.Alliance.RED
                    : PrecisionShooterSubsystem.Alliance.BLUE;
        }

        hood.setAngleDegrees(manualHoodDeg);
        hood.update();
        flywheel.setTargetRpm(spinEnabled ? manualRpm : 0.0);
        flywheel.update(5000.0, config.nominalBatteryVoltage, config.flywheelIntegralLimit);

        if (gamepad1.a && !prevA) {
            Pose pose = follower.getPose();
            double goalX = alliance == PrecisionShooterSubsystem.Alliance.BLUE ? config.blueGoalXInches : config.redGoalXInches;
            double goalY = alliance == PrecisionShooterSubsystem.Alliance.BLUE ? config.blueGoalYInches : config.redGoalYInches;
            double distance = Math.hypot(goalX - pose.getX(), goalY - pose.getY());
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
        prevRightBumper = gamepad1.right_bumper;
        prevX = gamepad1.x;
        prevY = gamepad1.y;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Spin", spinEnabled);
        telemetry.addData("Manual RPM", "%.1f", manualRpm);
        telemetry.addData("Actual RPM", "%.1f", flywheel.getMeasuredRpm());
        telemetry.addData("Hood Deg", "%.2f", manualHoodDeg);
        telemetry.addData("Samples", samples.size());
        telemetry.addLine("A = record, X = delete last, B = print Panels rows, Y = swap alliance");
        if (!samples.isEmpty()) {
            PrecisionShotTable.Entry last = samples.get(samples.size() - 1);
            telemetry.addData("Last Sample", Arrays.asList(last.distanceInches, last.targetRpm, last.hoodAngleDeg));
        }
        telemetry.update();
    }

    private void emitTable() {
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
        builder.append("PrecisionShooterConfig Panels rows:\n");
        for (int i = 0; i < 8; i++) {
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
}
