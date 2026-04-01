package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.PedroConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TeleopConstants;
import org.firstinspires.ftc.teamcode.Subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "TUNE_SHOOTER_SYSTEM", group = "Tuning")
public class PrecisionShooterTuningTeleOp extends OpMode {

    private static final String LOG_TAG = "PrecisionShotCsv";

    private enum ShootFeedState {
        IDLE,
        ARMED_WAITING_READY,
        GATE_OPENING,
        FEEDING
    }

    private final ShooterConstants config = new ShooterConstants();
    private final ElapsedTime adjustTimer = new ElapsedTime();
    private final ElapsedTime blockerOpenTimer = new ElapsedTime();

    private Follower follower;
    private PrecisionShooterSubsystem shooter;
    private IntakeMotor intake;
    private PrecisionShooterSubsystem.Alliance alliance = PrecisionShooterSubsystem.Alliance.BLUE;
    private final ShotCsvLogger shotLogger = new ShotCsvLogger();
    private boolean feedingEnabled;
    private boolean prevB;
    private boolean prevRightStickButton;
    private boolean prevX;
    private boolean prevY;
    private double goalX;
    private double goalY;
    private long lastLoopNs;
    private ShootFeedState shootFeedState = ShootFeedState.IDLE;

    @Override
    public void init() {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.update();
        Indexer_Base indexerBase = new Indexer_Base(hardwareMap);
        indexerBase.StartIndexPose();
        intake = indexerBase.intkM;
        shooter = PrecisionShooterSubsystem.create(hardwareMap, follower, config);
        applyAllianceDefaults();
        adjustTimer.reset();
        blockerOpenTimer.reset();
    }

    @Override
    public void start() {
        shooter.start();
        setShootFeedState(ShootFeedState.IDLE);
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
            setShootFeedState(feedingEnabled ? ShootFeedState.IDLE : ShootFeedState.ARMED_WAITING_READY);
        }
        if (gamepad1.x && !prevX) {
            if (feedingEnabled) {
                setShootFeedState(ShootFeedState.IDLE);
            }
        }
        if (gamepad1.b && !prevB) {
            alliance = alliance == PrecisionShooterSubsystem.Alliance.BLUE
                    ? PrecisionShooterSubsystem.Alliance.RED
                    : PrecisionShooterSubsystem.Alliance.BLUE;
            applyAllianceDefaults();
        }
        if (gamepad1.right_stick_button && !prevRightStickButton) {
            if (shotLogger.isRecording()) {
                shotLogger.stop();
            } else {
                shotLogger.start();
            }
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

        long nowNs = System.nanoTime();
        double loopMs = lastLoopNs == 0L ? 0.0 : (nowNs - lastLoopNs) / 1_000_000.0;
        lastLoopNs = nowNs;

        prevB = gamepad1.b;
        prevRightStickButton = gamepad1.right_stick_button;
        prevX = gamepad1.x;
        prevY = gamepad1.y;

        shooter.setAlliance(alliance);
        shooter.setGoalPosition(goalX, goalY);
        shooter.setSpinEnabled(TeleopConstants.ALWAYS_SPIN_FLYWHEEL || feedingEnabled);
        shooter.setAutoAimEnabled(true);
        shooter.requestFire(feedingEnabled);
        shooter.update();

        PrecisionShooterSubsystem.TelemetrySnapshot snapshot = shooter.snapshot();
        updateShootFeedState(shooter.isFeedGateOpen());
        updateFeedAssist();
        shotLogger.logSample(
                getRuntime(),
                loopMs,
                follower.getPose(),
                goalX,
                goalY,
                alliance,
                TeleopConstants.ALWAYS_SPIN_FLYWHEEL || feedingEnabled,
                true,
                shooter.isFireRequested(),
                shooter.isFeedGateOpen(),
                shooter.getFlywheelErrorRpm(),
                shooter.getFlywheelControlPower(),
                shooter.getBatteryVoltage(),
                snapshot
        );

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Goal", "(%.1f, %.1f)", goalX, goalY);
        telemetry.addData("Aim Goal", "(%.1f, %.1f)", snapshot.aimGoalX, snapshot.aimGoalY);
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Shoot Armed", feedingEnabled);
        telemetry.addData("Feed State", shootFeedState);
        telemetry.addData("Always Spin", TeleopConstants.ALWAYS_SPIN_FLYWHEEL);
        telemetry.addData("Auto Aim", true);
        telemetry.addData("Homed", snapshot.homed);
        telemetry.addData("Shot Zone", snapshot.inShootingZone);
        telemetry.addData("Ready", snapshot.ready);
        telemetry.addData("Target RPM", "%.1f", snapshot.targetRpm);
        telemetry.addData("Actual RPM", "%.1f", snapshot.actualRpm);
        telemetry.addData("RPM Error", "%.1f", shooter.getFlywheelErrorRpm());
        telemetry.addData("Flywheel Power", "%.3f", shooter.getFlywheelControlPower());
        telemetry.addData("Battery", "%.2f", shooter.getBatteryVoltage());
        telemetry.addData("Comp RPM", "%.1f", snapshot.compensationRpm);
        telemetry.addData("Nominal Hood", "%.2f", snapshot.nominalHoodDeg);
        telemetry.addData("Comp Hood", "%.2f", snapshot.compensatedHoodDeg);
        telemetry.addData("Hood Delta", "%.2f", snapshot.compensatedHoodDeg - snapshot.nominalHoodDeg);
        telemetry.addData("Goal Fwd Offset", "%.2f", snapshot.goalForwardOffsetInches);
        telemetry.addData("Turret", "%.2f / %.2f", snapshot.turretAngleDeg, snapshot.turretTargetDeg);
        telemetry.addData("Predicted Range", "%.2f", snapshot.predictedRangeInches);
        telemetry.addData("TOF", "%.3f", snapshot.timeOfFlightSeconds);
        telemetry.addData("Feed Gate", shooter.isFeedGateOpen());
        telemetry.addData("Shot Log", shotLogger.getStatusLine());
        telemetry.addData("Status", snapshot.status);
        telemetry.addData("Feed Assist", gamepad1.left_bumper ? "Manual Intake" : (gamepad1.left_trigger > 0.2 ? "Reverse" : (shootFeedState == ShootFeedState.FEEDING ? "Auto Feed" : "Off")));
        telemetry.addLine("Y arm/cancel shot, X cancel shot, LB intake, LT reverse");
        telemetry.addLine("B alliance, A reset goal");
        telemetry.addLine("Dpad adjusts goal X/Y by 2 inches, RS click toggles CSV log");
        telemetry.update();
    }

    @Override
    public void stop() {
        setShootFeedState(ShootFeedState.IDLE);
        shotLogger.stop();
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

    private void updateFeedAssist() {
        if (intake == null) {
            return;
        }

        if (gamepad1.left_trigger > 0.2) {
            intake.outtake();
        } else if (gamepad1.left_bumper) {
            intake.intake();
        } else if (shootFeedState == ShootFeedState.FEEDING) {
            intake.slowIntake();
        } else {
            intake.stop();
        }
    }

    private void updateShootFeedState(boolean gateOpen) {
        if (!feedingEnabled) {
            shootFeedState = ShootFeedState.IDLE;
            return;
        }

        if (!gateOpen) {
            if (shootFeedState != ShootFeedState.ARMED_WAITING_READY) {
                shootFeedState = ShootFeedState.ARMED_WAITING_READY;
            }
            return;
        }

        if (shootFeedState == ShootFeedState.ARMED_WAITING_READY) {
            shootFeedState = ShootFeedState.GATE_OPENING;
            blockerOpenTimer.reset();
            return;
        }

        if (shootFeedState == ShootFeedState.GATE_OPENING
                && blockerOpenTimer.seconds() >= ShooterConstants.feedOpenSettlingSeconds) {
            shootFeedState = ShootFeedState.FEEDING;
        }
    }

    private void setShootFeedState(ShootFeedState nextState) {
        shootFeedState = nextState;
        feedingEnabled = nextState != ShootFeedState.IDLE;
        if (nextState != ShootFeedState.GATE_OPENING) {
            blockerOpenTimer.reset();
        }
    }

    private final class ShotCsvLogger {
        private static final String DIRECTORY_NAME = "shotlogs";

        private BufferedWriter writer;
        private File currentFile;
        private File lastSavedFile;
        private boolean lastFeedGateOpen;
        private int shotIndex;
        private int rowsSinceFlush;
        private String statusLine = "idle";

        boolean isRecording() {
            return writer != null;
        }

        String getStatusLine() {
            return statusLine;
        }

        void start() {
            stop();

            File directory = new File(AppUtil.ROBOT_DATA_DIR, DIRECTORY_NAME);
            if (!directory.exists() && !directory.mkdirs()) {
                statusLine = "mkdir failed";
                telemetry.log().add("Shot log directory failed: " + directory.getAbsolutePath());
                return;
            }

            currentFile = new File(directory, String.format(Locale.US,
                    "precision_shots_%d.csv",
                    System.currentTimeMillis()));
            try {
                writer = new BufferedWriter(new FileWriter(currentFile, false));
                writer.write("time_sec,loop_ms,shot_index,feed_edge,alliance,pose_x_in,pose_y_in,pose_heading_deg,goal_x_in,goal_y_in,spin_enabled,auto_aim_enabled,in_zone,ready,fire_requested,feed_gate_open,target_rpm,actual_rpm,comp_rpm,rpm_error,power_cmd,battery_v,nominal_hood_deg,commanded_hood_deg,hood_delta_deg,aim_goal_x_in,aim_goal_y_in,turret_angle_deg,turret_target_deg,predicted_range_in,time_of_flight_s,status");
                writer.newLine();
                writer.flush();
                lastFeedGateOpen = false;
                shotIndex = 0;
                rowsSinceFlush = 0;
                statusLine = "recording: " + currentFile.getName();
                telemetry.log().add("Shot log started: " + currentFile.getAbsolutePath());
            } catch (IOException e) {
                RobotLog.ee(LOG_TAG, e, "Failed to start shot log");
                statusLine = "open failed";
                telemetry.log().add("Shot log open failed.");
                closeQuietly();
            }
        }

        void logSample(double timeSeconds,
                       double loopMs,
                       Pose pose,
                       double currentGoalX,
                       double currentGoalY,
                       PrecisionShooterSubsystem.Alliance currentAlliance,
                       boolean currentSpinEnabled,
                       boolean currentAutoAimEnabled,
                       boolean fireRequested,
                       boolean feedGateOpen,
                       double rpmError,
                       double powerCommand,
                       double batteryVoltage,
                       PrecisionShooterSubsystem.TelemetrySnapshot snapshot) {
            if (writer == null) {
                return;
            }

            boolean feedEdge = feedGateOpen && !lastFeedGateOpen;
            if (feedEdge) {
                shotIndex++;
            }

            try {
                writer.write(String.format(Locale.US,
                        "%.3f,%.3f,%d,%d,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%b,%b,%b,%b,%b,%b,%.2f,%.2f,%.2f,%.2f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%s",
                        timeSeconds,
                        loopMs,
                        shotIndex,
                        feedEdge ? 1 : 0,
                        currentAlliance,
                        pose.getX(),
                        pose.getY(),
                        Math.toDegrees(pose.getHeading()),
                        currentGoalX,
                        currentGoalY,
                        currentSpinEnabled,
                        currentAutoAimEnabled,
                        snapshot.inShootingZone,
                        snapshot.ready,
                        fireRequested,
                        feedGateOpen,
                        snapshot.targetRpm,
                        snapshot.actualRpm,
                        snapshot.compensationRpm,
                        rpmError,
                        powerCommand,
                        batteryVoltage,
                        snapshot.nominalHoodDeg,
                        snapshot.compensatedHoodDeg,
                        snapshot.compensatedHoodDeg - snapshot.nominalHoodDeg,
                        snapshot.aimGoalX,
                        snapshot.aimGoalY,
                        snapshot.turretAngleDeg,
                        snapshot.turretTargetDeg,
                        snapshot.predictedRangeInches,
                        snapshot.timeOfFlightSeconds,
                        csvSafe(snapshot.status)));
                writer.newLine();
                rowsSinceFlush++;
                if (rowsSinceFlush >= 25 || feedEdge) {
                    writer.flush();
                    rowsSinceFlush = 0;
                }
            } catch (IOException e) {
                RobotLog.ee(LOG_TAG, e, "Failed to write shot log row");
                statusLine = "write failed";
                telemetry.log().add("Shot log write failed; logging stopped.");
                closeQuietly();
            }

            lastFeedGateOpen = feedGateOpen;
        }

        void stop() {
            if (writer == null) {
                if (lastSavedFile != null) {
                    statusLine = "saved: " + lastSavedFile.getName();
                }
                return;
            }

            try {
                writer.flush();
                writer.close();
                lastSavedFile = currentFile;
                statusLine = "saved: " + lastSavedFile.getName();
                telemetry.log().add("Shot log saved: " + lastSavedFile.getAbsolutePath());
            } catch (IOException e) {
                RobotLog.ee(LOG_TAG, e, "Failed to close shot log");
                statusLine = "close failed";
                telemetry.log().add("Shot log close failed.");
            } finally {
                writer = null;
                currentFile = null;
                rowsSinceFlush = 0;
            }
        }

        private void closeQuietly() {
            if (writer != null) {
                try {
                    writer.close();
                } catch (IOException ignored) {
                }
            }
            writer = null;
            currentFile = null;
            rowsSinceFlush = 0;
        }

        private String csvSafe(String value) {
            return value == null ? "" : value.replace(',', ';');
        }
    }
}
