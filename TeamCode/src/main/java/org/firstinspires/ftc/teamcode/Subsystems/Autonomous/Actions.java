package org.firstinspires.ftc.teamcode.Subsystems.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

public final class Actions {

    private Actions() {}

    public static AutoAction instant(Runnable runnable) {
        return new AutoAction() {
            private boolean done;

            @Override
            public void start() {
                runnable.run();
                done = true;
            }

            @Override
            public void update() {
            }

            @Override
            public boolean isFinished() {
                return done;
            }
        };
    }

    public static AutoAction waitUntil(BooleanSupplier condition) {
        return new AutoAction() {
            @Override
            public void start() {
            }

            @Override
            public void update() {
            }

            @Override
            public boolean isFinished() {
                return condition.getAsBoolean();
            }
        };
    }

    public static AutoAction waitSeconds(double seconds) {
        return new AutoAction() {
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public void start() {
                timer.reset();
            }

            @Override
            public void update() {
            }

            @Override
            public boolean isFinished() {
                return timer.seconds() >= seconds;
            }
        };
    }

    public static AutoAction followPath(Follower follower, PathChain path, double speed, boolean holdEnd) {
        return new AutoAction() {
            @Override
            public void start() {
                follower.followPath(path, speed, holdEnd);
            }

            @Override
            public void update() {
            }

            @Override
            public boolean isFinished() {
                return !follower.isBusy();
            }
        };
    }

    public static AutoAction sequence(AutoAction... actions) {
        return new SequenceAction(Arrays.asList(actions));
    }

    public static AutoAction parallel(AutoAction... actions) {
        return new ParallelAction(Arrays.asList(actions));
    }

    private static final class SequenceAction implements AutoAction {
        private final List<AutoAction> actions;
        private int index;
        private boolean startedCurrent;

        private SequenceAction(List<AutoAction> actions) {
            this.actions = new ArrayList<>(actions);
        }

        @Override
        public void start() {
            index = 0;
            startedCurrent = false;
        }

        @Override
        public void update() {
            if (index >= actions.size()) {
                return;
            }

            AutoAction current = actions.get(index);
            if (!startedCurrent) {
                current.start();
                startedCurrent = true;
            }

            current.update();
            if (current.isFinished()) {
                current.finish();
                index++;
                startedCurrent = false;
            }
        }

        @Override
        public boolean isFinished() {
            return index >= actions.size();
        }

        @Override
        public void finish() {
            if (index < actions.size() && startedCurrent) {
                actions.get(index).finish();
            }
        }
    }

    private static final class ParallelAction implements AutoAction {
        private final List<AutoAction> actions;
        private final boolean[] started;

        private ParallelAction(List<AutoAction> actions) {
            this.actions = new ArrayList<>(actions);
            this.started = new boolean[actions.size()];
        }

        @Override
        public void start() {
            Arrays.fill(started, false);
        }

        @Override
        public void update() {
            for (int i = 0; i < actions.size(); i++) {
                AutoAction action = actions.get(i);
                if (action == null) {
                    continue;
                }
                if (!started[i]) {
                    action.start();
                    started[i] = true;
                }
                action.update();
                if (action.isFinished()) {
                    action.finish();
                    actions.set(i, null);
                }
            }
        }

        @Override
        public boolean isFinished() {
            for (AutoAction action : actions) {
                if (action != null) {
                    return false;
                }
            }
            return true;
        }

        @Override
        public void finish() {
            for (int i = 0; i < actions.size(); i++) {
                AutoAction action = actions.get(i);
                if (action != null && started[i]) {
                    action.finish();
                    actions.set(i, null);
                }
            }
        }
    }
}
