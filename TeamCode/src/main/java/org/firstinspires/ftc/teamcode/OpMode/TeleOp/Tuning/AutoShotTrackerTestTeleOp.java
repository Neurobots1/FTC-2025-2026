package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tuning;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.HardwareMapConstants;
import org.firstinspires.ftc.teamcode.Constants.PedroConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterHardwareConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TeleopConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.ColorSensorDataLogger;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShotFeedCadenceController;

@TeleOp(name = "TEST_SHOT_LOGGER", group = "Tuning")
public class AutoShotTrackerTestTeleOp extends OpMode {

    private final ShooterConstants config = new ShooterConstants();
    private final ShotFeedCadenceController shotFeedController = new ShotFeedCadenceController();
    private final ElapsedTime adjustTimer = new ElapsedTime();

    private Follower follower;
    private PrecisionShooterSubsystem shooter;
    private IntakeMotor intake;
    private ColorSensorDataLogger colorSensorLogger;

    private boolean prevY;
    private boolean prevX;
    private boolean prevA;
    private boolean prevB;
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
        colorSensorLogger = new ColorSensorDataLogger(getColorSensor());

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
        if (colorSensorLogger != null) {
            colorSensorLogger.startSession();
        }
    }

    @Override
    public void loop() {
        handleButtons();
        updateRpmTrim();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

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
        if (colorSensorLogger != null) {
            colorSensorLogger.logSample(snapshot, shotFeedController, shooter.isFeedGateOpen());
        }

        updateIntake();

        telemetry.addData("Target RPM", "%.0f", snapshot.targetRpm);
        telemetry.addData("Actual RPM", "%.0f", snapshot.actualRpm);
        telemetry.addData("Shoot", shotFeedController.isArmed() ? "ON" : "OFF");
        telemetry.addData("Ready", snapshot.ready);
        telemetry.addData("Log File", colorSensorLogger == null ? "OFF" : colorSensorLogger.getOutputFileName());
        telemetry.addData("Logger", buildLoggerStatus());
        telemetry.addLine("Y = shoot toggle, B = label shot, A = mark reset");
        telemetry.addLine("Dpad up/down = RPM, X = zero trim");
        telemetry.addLine("LB = intake, LT = reverse");
        telemetry.update();
    }

    @Override
    public void stop() {
        shotFeedController.setArmed(false);
        if (intake != null) {
            intake.stop();
        }
        if (colorSensorLogger != null) {
            colorSensorLogger.close();
        }
    }

    private void handleButtons() {
        if (gamepad1.y && !prevY) {
            shotFeedController.setArmed(!shotFeedController.isArmed());
        }

        if (gamepad1.a && !prevA) {
            if (colorSensorLogger != null) {
                colorSensorLogger.markCounterReset();
            }
        }

        if (gamepad1.b && !prevB) {
            if (colorSensorLogger != null) {
                colorSensorLogger.markManualShot();
            }
        }

        if (gamepad1.x && !prevX) {
            rpmTrim = 0.0;
        }

        prevY = gamepad1.y;
        prevA = gamepad1.a;
        prevB = gamepad1.b;
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

    private RevColorSensorV3 getColorSensor() {
        try {
            return hardwareMap.get(RevColorSensorV3.class, HardwareMapConstants.COLOR_SENSOR);
        } catch (RuntimeException ignored) {
            return null;
        }
    }

    private String buildLoggerStatus() {
        if (colorSensorLogger == null) {
            return "UNAVAILABLE";
        }

        if (!colorSensorLogger.isLogging()) {
            String error = colorSensorLogger.getLastError();
            return error == null || error.isEmpty() ? "OFF" : "ERR: " + error;
        }

        return colorSensorLogger.hasColorSensor() ? "ACTIVE" : "ACTIVE (no sensor)";
    }
}
