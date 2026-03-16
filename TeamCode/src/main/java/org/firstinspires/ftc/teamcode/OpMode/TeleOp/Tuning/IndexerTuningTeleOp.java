package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.IndexerMode;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PGP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.PatternMappedIndexer;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.SortPattern;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TUNE_INDEXER", group = "Tuning")
public class IndexerTuningTeleOp extends OpMode {

    private Follower follower;
    private LauncherSubsystem launcher;
    private Indexer_PGP physicalIndexer;
    private IndexerMode activeIndexer;
    private SortPattern activePattern = SortPattern.PGP;
    private AllianceSelector.Alliance alliance = AllianceSelector.Alliance.BLUE;

    private boolean preSpinEnabled;
    private boolean prevA;
    private boolean prevB;
    private boolean prevX;
    private boolean prevY;
    private boolean prevDpadUp;
    private boolean prevDpadDown;
    private boolean prevDpadLeft;
    private boolean prevDpadRight;
    private boolean prevLeftBumper;
    private boolean prevOptions;
    private boolean prevShare;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        DcMotorEx flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        DcMotorEx flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcher = new LauncherSubsystem(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        launcher.setBlocker(hardwareMap.get(Servo.class, "Blocker"));
        launcher.init();

        Indexer_Base indexerBase = new Indexer_Base(hardwareMap);
        indexerBase.StartIndexPose();
        physicalIndexer = new Indexer_PGP(hardwareMap, indexerBase, launcher);
        activeIndexer = physicalIndexer;
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
                false,
                AllianceSelector.Field.fieldCentricOffset(alliance)
        );

        Pose pose = follower.getPose();
        double goalX = AllianceSelector.Field.goalX(alliance);
        double goalY = AllianceSelector.Field.goalY(alliance);
        double distance = Math.hypot(goalX - pose.getX(), goalY - pose.getY());

        handleDriverInputs();

        activeIndexer.setShootContext(pose.getX(), pose.getY(), distance);
        activeIndexer.setPreSpinEnabled(preSpinEnabled);
        activeIndexer.update();

        telemetry.addData("Pattern", activePattern.displayName());
        telemetry.addData("Alliance", alliance);
        telemetry.addData("PreSpin", preSpinEnabled);
        telemetry.addData("Busy", activeIndexer.isBusy());
        telemetry.addData("Active Seq", physicalIndexer.getActiveSequenceSummary());
        telemetry.addData("Color mm", "%.1f", physicalIndexer.getColorDistanceMm());
        telemetry.addData("Want Shoot", physicalIndexer.isShootRequested());
        telemetry.addData("Launcher Target", "%.1f", launcher.getTargetTPS());
        telemetry.addData("Launcher RPM", "%.1f", launcher.getCurrentRPM());
        telemetry.addData("Shooter Ready", launcher.flywheelReady());
        telemetry.addData("Pose", pose);
        telemetry.addLine("A/B/X intake L1/L2/L3");
        telemetry.addLine("Dpad Left/Down/Right shoot L1/L2/L3, Dpad Up preload");
        telemetry.addLine("LB pre-spin, Y pattern, Share alliance, Options stop");
        telemetry.update();
    }

    private void handleDriverInputs() {
        if (gamepad1.a && !prevA) {
            activeIndexer.startIntake(1);
        }
        if (gamepad1.b && !prevB) {
            activeIndexer.startIntake(2);
        }
        if (gamepad1.x && !prevX) {
            activeIndexer.startIntake(3);
        }
        if (gamepad1.dpad_left && !prevDpadLeft) {
            activeIndexer.startOuttake(1);
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            activeIndexer.startOuttake(2);
        }
        if (gamepad1.dpad_right && !prevDpadRight) {
            activeIndexer.startOuttake(3);
        }
        if (gamepad1.dpad_up && !prevDpadUp) {
            activeIndexer.startOuttake(4);
        }
        if (gamepad1.left_bumper && !prevLeftBumper) {
            preSpinEnabled = !preSpinEnabled;
        }
        if (gamepad1.y && !prevY && !activeIndexer.isBusy()) {
            cyclePattern();
        }
        if (gamepad1.share && !prevShare && !activeIndexer.isBusy()) {
            alliance = alliance == AllianceSelector.Alliance.BLUE
                    ? AllianceSelector.Alliance.RED
                    : AllianceSelector.Alliance.BLUE;
        }
        if (gamepad1.options && !prevOptions) {
            activeIndexer.stopAll();
            preSpinEnabled = false;
        }

        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevLeftBumper = gamepad1.left_bumper;
        prevOptions = gamepad1.options;
        prevShare = gamepad1.share;
    }

    private void cyclePattern() {
        SortPattern[] patterns = {
                SortPattern.PGP,
                SortPattern.GPP,
                SortPattern.PPG,
                SortPattern.NOSORT
        };

        int nextIndex = 0;
        for (int i = 0; i < patterns.length; i++) {
            if (patterns[i] == activePattern) {
                nextIndex = (i + 1) % patterns.length;
                break;
            }
        }

        activePattern = patterns[nextIndex];
        if (activePattern == SortPattern.PGP) {
            activeIndexer = physicalIndexer;
        } else {
            activeIndexer = new PatternMappedIndexer(physicalIndexer, activePattern);
        }
    }
}
