package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

@SuppressWarnings("all")
public class Indexer_GPP implements IndexerMode {

    private final IndexerMode base;

    // GPP Line1 = PGP Line3
    // GPP Line2 = PGP Line1
    // GPP Line3 = PGP Line2
    private static final int[] MAP = {0, 3, 1, 2};

    public Indexer_GPP(IndexerMode base) {
        this.base = base;
    }

    private int map(int line) {
        if (line < 1 || line > 3) return line;
        return MAP[line];
    }

    @Override
    public void startIntake(int line) {
        base.startIntake(map(line));
    }

    @Override
    public void startOuttake(int line) {
        base.startOuttake(map(line));
    }

    @Override
    public boolean isBusy() {
        return base.isBusy();
    }

    @Override
    public void update() {
        base.update();
    }

    @Override
    public void stopAll() {
        base.stopAll();
    }

    @Override
    public void setShootContext(double x, double y, double distance) {
        base.setShootContext(x, y, distance);
    }
}
