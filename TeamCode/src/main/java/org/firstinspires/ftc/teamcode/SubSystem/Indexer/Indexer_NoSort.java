package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

@SuppressWarnings("all")
public class Indexer_NoSort implements IndexerMode {

    // NoSort = PGP Line2
    private final Indexer_PGP pgp;

    public Indexer_NoSort(Indexer_PGP pgpCore) {
        this.pgp = pgpCore;
    }

    @Override
    public boolean isBusy() {
        return pgp != null && pgp.isBusy();
    }

    @Override
    public void setShootContext(double x, double y, double distance) {
        if (pgp != null) pgp.setShootContext(x, y, distance);
    }

    @Override
    public void startIntake(int line) {
        if (pgp != null) pgp.startLine2Intake();
    }

    @Override
    public void startOuttake(int line) {
        if (pgp != null) pgp.startLine2Outtake();
    }

    @Override
    public void stopAll() {
        if (pgp != null) pgp.stopAll();
    }

    @Override
    public void update() {
        if (pgp != null) pgp.update();
    }
}
