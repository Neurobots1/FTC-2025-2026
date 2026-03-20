package org.firstinspires.ftc.teamcode.Subsystems.Indexer;

import com.qualcomm.robotcore.util.ElapsedTime;

final class IndexerSequenceRunner {

    interface CompletionCondition {
        boolean test(IndexerSequenceRunner routine);
    }

    static final class Step {
        final String name;
        final Runnable onEnter;
        final CompletionCondition completionCondition;
        final boolean resetTimerOnEnter;

        Step(String name,
             Runnable onEnter,
             CompletionCondition completionCondition,
             boolean resetTimerOnEnter) {
            this.name = name;
            this.onEnter = onEnter;
            this.completionCondition = completionCondition;
            this.resetTimerOnEnter = resetTimerOnEnter;
        }
    }

    private final String sequenceName;
    private final Step[] steps;
    private final ElapsedTime stepTimer = new ElapsedTime();

    private int currentStepIndex = -1;
    private boolean running;

    IndexerSequenceRunner(String sequenceName, Step... steps) {
        this.sequenceName = sequenceName;
        this.steps = steps;
    }

    void start() {
        if (steps.length == 0) {
            running = false;
            currentStepIndex = -1;
            return;
        }

        running = true;
        enterStep(0);
    }

    void update() {
        while (running && currentStepIndex >= 0 && currentStepIndex < steps.length) {
            Step step = steps[currentStepIndex];
            if (!step.completionCondition.test(this)) {
                return;
            }
            enterStep(currentStepIndex + 1);
        }
    }

    void stop() {
        running = false;
        currentStepIndex = -1;
    }

    boolean isRunning() {
        return running;
    }

    double secondsInStep() {
        return stepTimer.seconds();
    }

    String getSequenceName() {
        return sequenceName;
    }

    String getCurrentStepName() {
        if (!running || currentStepIndex < 0 || currentStepIndex >= steps.length) {
            return "IDLE";
        }
        return steps[currentStepIndex].name;
    }

    static Step step(String name,
                     Runnable onEnter,
                     CompletionCondition completionCondition,
                     boolean resetTimerOnEnter) {
        return new Step(name, onEnter, completionCondition, resetTimerOnEnter);
    }

    private void enterStep(int nextStepIndex) {
        if (nextStepIndex >= steps.length) {
            running = false;
            currentStepIndex = -1;
            return;
        }

        currentStepIndex = nextStepIndex;
        Step step = steps[currentStepIndex];
        if (step.onEnter != null) {
            step.onEnter.run();
        }
        if (step.resetTimerOnEnter) {
            stepTimer.reset();
        }
    }
}
