package org.firstinspires.ftc.teamcode.SubSystem.Autonomous;

public final class ActionScheduler {

    private AutoAction action;
    private boolean started;

    public void setAction(AutoAction action) {
        if (this.action != null) {
            this.action.finish();
        }
        this.action = action;
        this.started = false;
    }

    public void update() {
        if (action == null) {
            return;
        }

        if (!started) {
            action.start();
            started = true;
        }

        action.update();

        if (action.isFinished()) {
            action.finish();
            action = null;
            started = false;
        }
    }

    public boolean isIdle() {
        return action == null;
    }
}
