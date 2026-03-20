package org.firstinspires.ftc.teamcode.Subsystems.Indexer;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.firstinspires.ftc.teamcode.Constants.IndexerConstants;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;

final class IndexerScriptBuilder {

    private final IndexerRoutineBuilder delegate;
    private final IntakeMotor intake;
    private final Consumer<Double> setLeft;
    private final Consumer<Double> setRight;
    private final Consumer<Double> setBackGate;
    private final BooleanSupplier colorDetected;
    private final BooleanSupplier shooterReady;

    private IndexerScriptBuilder(String sequenceName,
                                 IntakeMotor intake,
                                 Consumer<Double> setLeft,
                                 Consumer<Double> setRight,
                                 Consumer<Double> setBackGate,
                                 BooleanSupplier colorDetected,
                                 BooleanSupplier shooterReady) {
        this.delegate = IndexerRoutineBuilder.sequence(sequenceName);
        this.intake = intake;
        this.setLeft = setLeft;
        this.setRight = setRight;
        this.setBackGate = setBackGate;
        this.colorDetected = colorDetected;
        this.shooterReady = shooterReady;
    }

    static IndexerScriptBuilder script(String sequenceName,
                                       IntakeMotor intake,
                                       Consumer<Double> setLeft,
                                       Consumer<Double> setRight,
                                       Consumer<Double> setBackGate,
                                       BooleanSupplier colorDetected,
                                       BooleanSupplier shooterReady) {
        return new IndexerScriptBuilder(
                sequenceName,
                intake,
                setLeft,
                setRight,
                setBackGate,
                colorDetected,
                shooterReady
        );
    }

    StepScriptBuilder step(String stepName) {
        return new StepScriptBuilder(delegate.step(stepName));
    }

    IndexerSequenceRunner build() {
        return delegate.build();
    }

    final class StepScriptBuilder {
        private final IndexerRoutineBuilder.StepBuilder delegateStep;

        StepScriptBuilder(IndexerRoutineBuilder.StepBuilder delegateStep) {
            this.delegateStep = delegateStep;
        }

        StepScriptBuilder action(Runnable action) {
            delegateStep.onEnter(action);
            return this;
        }

        StepScriptBuilder intakeOn() {
            return action(intake::intake);
        }

        StepScriptBuilder slowFeed() {
            return action(intake::slowIntake);
        }

        StepScriptBuilder intakeOff() {
            return action(intake::stop);
        }

        StepScriptBuilder intakeOut() {
            return action(intake::outtake);
        }

        StepScriptBuilder leftEngaged() {
            return action(() -> setLeft.accept(IndexerConstants.LEFT_ENGAGED));
        }

        StepScriptBuilder leftRetracted() {
            return action(() -> setLeft.accept(IndexerConstants.LEFT_RETRACTED));
        }

        StepScriptBuilder rightEngaged() {
            return action(() -> setRight.accept(IndexerConstants.RIGHT_ENGAGED));
        }

        StepScriptBuilder rightRetracted() {
            return action(() -> setRight.accept(IndexerConstants.RIGHT_RETRACTED));
        }

        StepScriptBuilder rightBlocker() {
            return action(() -> setRight.accept(IndexerConstants.RIGHT_BLOCKER));
        }

        StepScriptBuilder backGateOpen() {
            return action(() -> setBackGate.accept(IndexerConstants.BACK_GATE_OPEN));
        }

        StepScriptBuilder backGateClosed() {
            return action(() -> setBackGate.accept(IndexerConstants.BACK_GATE_CLOSED));
        }

        StepScriptBuilder resetTimer() {
            delegateStep.resetTimer();
            return this;
        }

        IndexerScriptBuilder waitSeconds(double seconds) {
            delegateStep.resetTimer();
            delegateStep.untilSeconds(seconds);
            return IndexerScriptBuilder.this;
        }

        IndexerScriptBuilder waitSeconds(DoubleSupplier secondsSupplier) {
            delegateStep.resetTimer();
            delegateStep.until(routine -> routine.secondsInStep() > secondsSupplier.getAsDouble());
            return IndexerScriptBuilder.this;
        }

        IndexerScriptBuilder waitSecondsInclusive(double seconds) {
            delegateStep.resetTimer();
            delegateStep.untilSecondsInclusive(seconds);
            return IndexerScriptBuilder.this;
        }

        IndexerScriptBuilder waitSecondsInclusive(DoubleSupplier secondsSupplier) {
            delegateStep.resetTimer();
            delegateStep.until(routine -> routine.secondsInStep() >= secondsSupplier.getAsDouble());
            return IndexerScriptBuilder.this;
        }

        IndexerScriptBuilder waitForColor() {
            delegateStep.until(routine -> colorDetected.getAsBoolean());
            return IndexerScriptBuilder.this;
        }

        IndexerScriptBuilder waitForShooterReady() {
            delegateStep.until(routine -> shooterReady.getAsBoolean());
            return IndexerScriptBuilder.this;
        }

        IndexerScriptBuilder waitUntil(BooleanSupplier condition) {
            delegateStep.until(routine -> condition.getAsBoolean());
            return IndexerScriptBuilder.this;
        }

        IndexerScriptBuilder done() {
            delegateStep.immediate();
            return IndexerScriptBuilder.this;
        }
    }
}
