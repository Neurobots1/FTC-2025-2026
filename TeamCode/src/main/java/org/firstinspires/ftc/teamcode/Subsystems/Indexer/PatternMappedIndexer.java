package org.firstinspires.ftc.teamcode.Subsystems.Indexer;

public class PatternMappedIndexer implements IndexerMode {

    private final IndexerMode base;
    private final SortPattern pattern;

    public PatternMappedIndexer(IndexerMode base, SortPattern pattern) {
        this.base = base;
        this.pattern = pattern;
    }

    @Override
    public void startIntake(int line) {
        base.startIntake(pattern.physicalLineFor(line));
    }

    @Override
    public void startOuttake(int line) {
        base.startOuttake(pattern.physicalLineFor(line));
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

    @Override
    public void setPreSpinEnabled(boolean enabled) {
        base.setPreSpinEnabled(enabled);
    }

    @Override
    public boolean isInShootingZone() {
        return base.isInShootingZone();
    }

    @Override
    public boolean isReadyToShoot() {
        return base.isReadyToShoot();
    }

    public SortPattern getPattern() {
        return pattern;
    }
}
