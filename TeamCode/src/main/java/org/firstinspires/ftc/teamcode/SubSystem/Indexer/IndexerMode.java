package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

public interface IndexerMode {

    void startIntake(int line);
    void startOuttake(int line);

    boolean isBusy();

    void update();
    void stopAll();

    void setShootContext(double x, double y, double distance);
}
