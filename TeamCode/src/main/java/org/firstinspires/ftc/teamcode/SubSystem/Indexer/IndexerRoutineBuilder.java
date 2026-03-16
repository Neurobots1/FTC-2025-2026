package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import java.util.ArrayList;
import java.util.List;

final class IndexerRoutineBuilder {

    private final String sequenceName;
    private final List<IndexerSequenceRunner.Step> steps = new ArrayList<>();

    private IndexerRoutineBuilder(String sequenceName) {
        this.sequenceName = sequenceName;
    }

    static IndexerRoutineBuilder sequence(String sequenceName) {
        return new IndexerRoutineBuilder(sequenceName);
    }

    StepBuilder step(String stepName) {
        return new StepBuilder(stepName);
    }

    IndexerSequenceRunner build() {
        return new IndexerSequenceRunner(sequenceName, steps.toArray(new IndexerSequenceRunner.Step[0]));
    }

    final class StepBuilder {
        private final String stepName;
        private Runnable onEnter;
        private IndexerSequenceRunner.CompletionCondition completionCondition = routine -> false;
        private boolean resetTimerOnEnter;

        StepBuilder(String stepName) {
            this.stepName = stepName;
        }

        StepBuilder onEnter(Runnable onEnter) {
            if (this.onEnter == null) {
                this.onEnter = onEnter;
            } else {
                Runnable existing = this.onEnter;
                this.onEnter = () -> {
                    existing.run();
                    onEnter.run();
                };
            }
            return this;
        }

        StepBuilder resetTimer() {
            this.resetTimerOnEnter = true;
            return this;
        }

        IndexerRoutineBuilder until(IndexerSequenceRunner.CompletionCondition completionCondition) {
            this.completionCondition = completionCondition;
            steps.add(new IndexerSequenceRunner.Step(stepName, onEnter, this.completionCondition, resetTimerOnEnter));
            return IndexerRoutineBuilder.this;
        }

        IndexerRoutineBuilder untilSeconds(double seconds) {
            return until(routine -> routine.secondsInStep() > seconds);
        }

        IndexerRoutineBuilder untilSecondsInclusive(double seconds) {
            return until(routine -> routine.secondsInStep() >= seconds);
        }

        IndexerRoutineBuilder immediate() {
            return until(routine -> true);
        }
    }
}
