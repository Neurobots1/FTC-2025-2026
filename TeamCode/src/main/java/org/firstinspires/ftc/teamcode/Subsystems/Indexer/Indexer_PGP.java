package org.firstinspires.ftc.teamcode.Subsystems.Indexer;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.HardwareMapConstants;
import org.firstinspires.ftc.teamcode.Constants.IndexerConstants;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;

import java.util.function.DoubleSupplier;

@Configurable
@SuppressWarnings("all")
public class Indexer_PGP implements IndexerMode {

    public static double COLOR_DETECT_MM = 60.0;

    public static double LINE1_INTAKE_SWAP_TO_MIDDLE_DELAY_S = 0.1;
    public static double LINE1_INTAKE_FINISH_STOP_S = 1.5;

    public static double LINE1_OUTTAKE_START_TO_SWAP_DELAY_S = 0.8;
    public static double LINE1_OUTTAKE_WAIT_S = 1.0;
    public static double LINE1_OUTTAKE_SWAP_TO_LEFT_DONE_S = 1.0;

    public static double LINE2_INTAKE_WAIT_S = 0.75;
    public static double LINE2_INTAKE_FINISH_STOP_S = 1.0;

    public static double LINE2_OUTTAKE_START_TO_SWAP_DELAY_S = 0.5;
    public static double LINE2_OUTTAKE_SWAP_TO_LEFT_DELAY_S = 0.45;
    public static double LINE2_OUTTAKE_FINISH_DONE_S = 0.5;

    public static double LINE3_INTAKE_SWAP_TO_RIGHT_DELAY_S = 0.15;
    public static double LINE3_INTAKE_SWAP_TO_LEFT_DELAY_S = 0.5;
    public static double LINE3_INTAKE_FINISH_STOP_S = 1.5;

    public static double LINE3_OUTTAKE_START_TO_SWAP_DELAY_S = 0.5;
    public static double LINE3_OUTTAKE_SWAP_TO_LEFT_DELAY_S = 0.45;
    public static double LINE3_OUTTAKE_FINISH_DONE_S = 0.5;

    public static double PRELOAD_OUTTAKE_START_DELAY_S = 1.5;
    public static double PRELOAD_OUTTAKE_FINISH_DONE_S = 1.5;

    private final Indexer_Base indexerBase;
    private final IntakeMotor intake;
    private final RevColorSensorV3 colorSensor;
    private final PrecisionShooterSubsystem shooter;

    private final IndexerSequenceRunner line1Intake;
    private final IndexerSequenceRunner line2Intake;
    private final IndexerSequenceRunner line3Intake;
    private final IndexerSequenceRunner line1Outtake;
    private final IndexerSequenceRunner line2Outtake;
    private final IndexerSequenceRunner line3Outtake;
    private final IndexerSequenceRunner preloadOuttake;
    private final IndexerSequenceRunner[] routines;

    private boolean wantShoot;
    private boolean wantPreSpin;

    private double shootX;
    private double shootY;
    private double shootDistance;

    public static double PRESPIN_TPS = 820;

    public Indexer_PGP(HardwareMap hardwareMap, Indexer_Base base, PrecisionShooterSubsystem shooter) {
        this.indexerBase = base;
        this.intake = base.intkM;
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, HardwareMapConstants.COLOR_SENSOR);
        this.shooter = shooter;

        line1Intake = buildLine1Intake();
        line2Intake = buildLine2Intake();
        line3Intake = buildLine3Intake();
        line1Outtake = buildLine1Outtake();
        line2Outtake = buildSharedOuttake(
                "Line 2 Outtake",
                () -> LINE2_OUTTAKE_START_TO_SWAP_DELAY_S,
                () -> LINE2_OUTTAKE_SWAP_TO_LEFT_DELAY_S,
                () -> LINE2_OUTTAKE_FINISH_DONE_S
        );
        line3Outtake = buildSharedOuttake(
                "Line 3 Outtake",
                () -> LINE3_OUTTAKE_START_TO_SWAP_DELAY_S,
                () -> LINE3_OUTTAKE_SWAP_TO_LEFT_DELAY_S,
                () -> LINE3_OUTTAKE_FINISH_DONE_S
        );
        preloadOuttake = buildPreloadOuttake();

        routines = new IndexerSequenceRunner[]{
                line1Intake,
                line2Intake,
                line3Intake,
                line1Outtake,
                line2Outtake,
                line3Outtake,
                preloadOuttake
        };
    }

    public void startPreSpin() {
        wantPreSpin = true;
    }

    @Override
    public void setPreSpinEnabled(boolean enabled) {
        wantPreSpin = enabled;
        if (!enabled && !wantShoot && !isBusy() && shooter != null) {
            shooter.setSpinEnabled(false);
            shooter.requestFire(false);
        }
    }

    public void stopPreSpinIfIdle() {
        if (!isBusy()) {
            wantPreSpin = false;
            wantShoot = false;
        }
    }

    @Override
    public boolean isBusy() {
        for (IndexerSequenceRunner routine : routines) {
            if (routine.isRunning()) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void setShootContext(double x, double y, double distance) {
        shootX = x;
        shootY = y;
        shootDistance = distance;
    }

    @Override
    public boolean isInShootingZone() {
        return shooter != null && shooter.isInShootingZone();
    }

    @Override
    public boolean isReadyToShoot() {
        return isInShootingZone() && shooterReady();
    }

    @Override
    public void startIntake(int line) {
        if (line == 1) {
            startLine1Intake();
        } else if (line == 2) {
            startLine2Intake();
        } else if (line == 3) {
            startLine3Intake();
        }
    }

    @Override
    public void startOuttake(int line) {
        if (line == 1) {
            startLine1Outtake();
        } else if (line == 2) {
            startLine2Outtake();
        } else if (line == 3) {
            startLine3Outtake();
        } else if (line == 4) {
            startLine4Outtake();
        }
    }

    public void startLine1Intake() {
        if (isBusy()) {
            return;
        }
        line1Intake.start();
    }

    public void startLine1Outtake() {
        if (isBusy()) {
            return;
        }
        beginShootSequence(line1Outtake);
    }

    public void startLine2Intake() {
        if (isBusy()) {
            return;
        }
        line2Intake.start();
    }

    public void startLine2Outtake() {
        if (isBusy()) {
            return;
        }
        beginShootSequence(line2Outtake);
    }

    public void startLine3Intake() {
        if (isBusy()) {
            return;
        }
        line3Intake.start();
    }

    public void startLine3Outtake() {
        if (isBusy()) {
            return;
        }
        beginShootSequence(line3Outtake);
    }

    public void startLine4Outtake() {
        if (isBusy()) {
            return;
        }
        beginShootSequence(preloadOuttake);
    }

    @Override
    public void stopAll() {
        for (IndexerSequenceRunner routine : routines) {
            routine.stop();
        }

        wantShoot = false;
        wantPreSpin = false;
        intake.stop();
        if (shooter != null) {
            shooter.setSpinEnabled(false);
            shooter.requestFire(false);
            shooter.update();
        }
        if (indexerBase != null) {
            indexerBase.StartIndexPose();
        }
    }

    @Override
    public void update() {
        updateShooterDemand();

        for (IndexerSequenceRunner routine : routines) {
            routine.update();
        }

        if (!isBusy() && !wantShoot && !wantPreSpin && shooter != null) {
            shooter.setSpinEnabled(false);
        }
    }

    public double getColorDistanceMm() {
        return colorSensor.getDistance(DistanceUnit.MM);
    }

    public boolean isShootRequested() {
        return wantShoot;
    }

    public boolean isPreSpinRequested() {
        return wantPreSpin;
    }

    public String getActiveSequenceSummary() {
        StringBuilder builder = new StringBuilder();
        for (IndexerSequenceRunner routine : routines) {
            if (!routine.isRunning()) {
                continue;
            }
            if (builder.length() > 0) {
                builder.append(" | ");
            }
            builder.append(routine.getSequenceName()).append(": ").append(routine.getCurrentStepName());
        }
        return builder.length() == 0 ? "IDLE" : builder.toString();
    }

    private void beginShootSequence(IndexerSequenceRunner routine) {
        wantPreSpin = false;
        wantShoot = true;
        intake.intake();
        routine.start();
    }

    private void updateShooterDemand() {
        if (shooter == null) {
            return;
        }

        if (wantShoot) {
            shooter.setSpinEnabled(true);
            shooter.setAutoAimEnabled(true);
            shooter.requestFire(true);
        } else if (wantPreSpin) {
            shooter.setSpinEnabled(true);
            shooter.setAutoAimEnabled(true);
            shooter.requestFire(false);
        } else {
            shooter.setSpinEnabled(false);
            shooter.setAutoAimEnabled(true);
            shooter.requestFire(false);
        }

        shooter.update();
    }

    private boolean colorDetected() {
        return getColorDistanceMm() <= COLOR_DETECT_MM;
    }

    private boolean shooterReady() {
        return shooter != null && shooter.isReadyToShootNow();
    }

    private void finishShootSequence() {
        intake.stop();
        wantShoot = false;
    }

    private void setLeft(double position) {
        indexerBase.indexLeftServo.setPosition(position);
    }

    private void setRight(double position) {
        indexerBase.indexRightServo.setPosition(position);
    }

    private void setBackGate(double position) {
        indexerBase.indexGateBack.setPosition(position);
    }

    private IndexerScriptBuilder script(String name) {
        return IndexerScriptBuilder.script(
                name,
                intake,
                this::setLeft,
                this::setRight,
                this::setBackGate,
                this::colorDetected,
                this::shooterReady
        );
    }

    private IndexerSequenceRunner buildLine1Intake() {
        return script("Line 1 Intake")
                .step("Capture")
                .intakeOn()
                .leftEngaged()
                .rightRetracted()
                .backGateClosed()
                .waitForColor()
                .step("Wait Swap To Middle")
                .waitSeconds(() -> LINE1_INTAKE_SWAP_TO_MIDDLE_DELAY_S)
                .step("Transfer Center")
                .rightEngaged()
                .leftRetracted()
                .waitSeconds(0.20)
                .step("Gate Open Hold")
                .backGateOpen()
                .waitSecondsInclusive(() -> LINE1_INTAKE_FINISH_STOP_S)
                .step("Cleanup")
                .rightBlocker()
                .intakeOff()
                .done()
                .build();
    }

    private IndexerSequenceRunner buildLine2Intake() {
        return script("Line 2 Intake")
                .step("Capture")
                .intakeOn()
                .rightEngaged()
                .backGateOpen()
                .waitForColor()
                .step("Wait Transfer")
                .waitSeconds(() -> LINE2_INTAKE_WAIT_S)
                .step("Transfer Left")
                .rightRetracted()
                .leftEngaged()
                .waitSecondsInclusive(() -> LINE2_INTAKE_FINISH_STOP_S)
                .step("Cleanup")
                .leftRetracted()
                .intakeOff()
                .done()
                .build();
    }

    private IndexerSequenceRunner buildLine3Intake() {
        return script("Line 3 Intake")
                .step("Capture")
                .intakeOn()
                .leftRetracted()
                .rightEngaged()
                .backGateClosed()
                .waitForColor()
                .step("Wait Swap Right")
                .waitSeconds(() -> LINE3_INTAKE_SWAP_TO_RIGHT_DELAY_S)
                .step("Swap Right")
                .rightRetracted()
                .leftEngaged()
                .waitSeconds(() -> LINE3_INTAKE_SWAP_TO_LEFT_DELAY_S)
                .step("Open Gate")
                .leftRetracted()
                .backGateOpen()
                .waitSecondsInclusive(() -> LINE3_INTAKE_FINISH_STOP_S)
                .step("Cleanup")
                .intakeOff()
                .done()
                .build();
    }

    private IndexerSequenceRunner buildLine1Outtake() {
        return script("Line 1 Outtake")
                .step("Wait Shooter Ready")
                .waitForShooterReady()
                .step("Slow Feed")
                .slowFeed()
                .waitSeconds(() -> LINE1_OUTTAKE_START_TO_SWAP_DELAY_S)
                .step("Swap Start")
                .leftRetracted()
                .rightEngaged()
                .backGateOpen()
                .waitSeconds(() -> LINE1_OUTTAKE_WAIT_S)
                .step("Swap To Left")
                .rightRetracted()
                .leftEngaged()
                .waitSecondsInclusive(() -> LINE1_OUTTAKE_SWAP_TO_LEFT_DONE_S)
                .step("Cleanup")
                .action(() -> {
                    setRight(IndexerConstants.RIGHT_RETRACTED);
                    setLeft(IndexerConstants.LEFT_RETRACTED);
                    setBackGate(IndexerConstants.BACK_GATE_OPEN);
                    finishShootSequence();
                })
                .done()
                .build();
    }

    private IndexerSequenceRunner buildSharedOuttake(String name,
                                                     DoubleSupplier startDelay,
                                                     DoubleSupplier swapToLeftDelay,
                                                     DoubleSupplier finishDelay) {
        return script(name)
                .step("Wait Shooter Ready")
                .waitForShooterReady()
                .step("Slow Feed")
                .slowFeed()
                .waitSeconds(startDelay)
                .step("Swap Start")
                .leftRetracted()
                .rightEngaged()
                .backGateOpen()
                .waitSecondsInclusive(swapToLeftDelay)
                .step("Swap To Left")
                .rightRetracted()
                .leftEngaged()
                .backGateOpen()
                .waitSecondsInclusive(finishDelay)
                .step("Cleanup")
                .action(() -> {
                    setRight(IndexerConstants.RIGHT_RETRACTED);
                    setLeft(IndexerConstants.LEFT_RETRACTED);
                    setBackGate(IndexerConstants.BACK_GATE_OPEN);
                    finishShootSequence();
                })
                .done()
                .build();
    }

    private IndexerSequenceRunner buildPreloadOuttake() {
        return script("Preload Outtake")
                .step("Wait Shooter Ready")
                .waitForShooterReady()
                .step("Slow Feed")
                .slowFeed()
                .waitSeconds(() -> PRELOAD_OUTTAKE_START_DELAY_S)
                .step("Pass Through")
                .leftRetracted()
                .rightRetracted()
                .backGateOpen()
                .waitSeconds(() -> PRELOAD_OUTTAKE_FINISH_DONE_S)
                .step("Cleanup")
                .action(this::finishShootSequence)
                .done()
                .build();
    }
}
