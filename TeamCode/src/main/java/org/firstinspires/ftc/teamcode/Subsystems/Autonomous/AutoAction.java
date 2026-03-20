package org.firstinspires.ftc.teamcode.Subsystems.Autonomous;

public interface AutoAction {

    void start();

    void update();

    boolean isFinished();

    default void finish() {}
}
