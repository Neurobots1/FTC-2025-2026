package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

@SuppressWarnings("all")
public class Indexer_PPG implements IndexerMode {

    private final IndexerMode base;

    // PPG Line1 = PGP Line2
    // PPG Line2 = PGP Line3
    // PPG Line3 = PGP Line1
    private static final int[] MAP = {0, 2, 3, 1};

    public Indexer_PPG(IndexerMode base) {
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
