package org.firstinspires.ftc.teamcode.SubSystem.Autonomous;

public interface AutoAction {

    void start();

    void update();

    boolean isFinished();

    default void finish() {}
}
