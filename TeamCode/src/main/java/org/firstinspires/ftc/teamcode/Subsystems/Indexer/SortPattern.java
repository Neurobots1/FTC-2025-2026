package org.firstinspires.ftc.teamcode.Subsystems.Indexer;

public enum SortPattern {
    PGP("PGP", new int[]{0, 1, 2, 3}),
    GPP("GPP", new int[]{0, 3, 1, 2}),
    PPG("PPG", new int[]{0, 2, 3, 1}),
    NOSORT("NoSort", new int[]{0, 2, 2, 2});

    private final String displayName;
    private final int[] logicalToPhysicalLine;

    SortPattern(String displayName, int[] logicalToPhysicalLine) {
        this.displayName = displayName;
        this.logicalToPhysicalLine = logicalToPhysicalLine;
    }

    public String displayName() {
        return displayName;
    }

    public int physicalLineFor(int logicalLine) {
        if (logicalLine < 1 || logicalLine >= logicalToPhysicalLine.length) {
            return logicalLine;
        }
        return logicalToPhysicalLine[logicalLine];
    }
}
