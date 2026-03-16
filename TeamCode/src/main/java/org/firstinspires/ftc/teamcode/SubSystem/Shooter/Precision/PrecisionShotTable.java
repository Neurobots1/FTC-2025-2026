package org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

public final class PrecisionShotTable {

    public static final class Entry {
        public final double distanceInches;
        public final double targetRpm;
        public final double hoodAngleDeg;

        public Entry(double distanceInches, double targetRpm, double hoodAngleDeg) {
            this.distanceInches = distanceInches;
            this.targetRpm = targetRpm;
            this.hoodAngleDeg = hoodAngleDeg;
        }
    }

    private final List<Entry> entries;

    public PrecisionShotTable(List<Entry> entries) {
        ArrayList<Entry> sorted = new ArrayList<>(entries);
        sorted.sort(Comparator.comparingDouble(e -> e.distanceInches));
        this.entries = Collections.unmodifiableList(sorted);
    }

    public Entry sample(double distanceInches) {
        if (entries.isEmpty()) {
            return new Entry(distanceInches, 0.0, 40.0);
        }
        if (distanceInches <= entries.get(0).distanceInches) {
            return entries.get(0);
        }
        if (distanceInches >= entries.get(entries.size() - 1).distanceInches) {
            return entries.get(entries.size() - 1);
        }
        for (int i = 0; i < entries.size() - 1; i++) {
            Entry a = entries.get(i);
            Entry b = entries.get(i + 1);
            if (distanceInches >= a.distanceInches && distanceInches <= b.distanceInches) {
                double t = (distanceInches - a.distanceInches) / (b.distanceInches - a.distanceInches);
                return new Entry(
                        distanceInches,
                        ShooterMath.lerp(a.targetRpm, b.targetRpm, t),
                        ShooterMath.lerp(a.hoodAngleDeg, b.hoodAngleDeg, t)
                );
            }
        }
        return entries.get(entries.size() - 1);
    }

    public List<Entry> getEntries() {
        return entries;
    }

    public String toJavaInitializer(String tableName) {
        StringBuilder builder = new StringBuilder();
        builder.append("public static final PrecisionShotTable ")
                .append(tableName)
                .append(" = new PrecisionShotTable(Arrays.asList(\n");
        for (int i = 0; i < entries.size(); i++) {
            Entry entry = entries.get(i);
            builder.append(String.format(Locale.US,
                    "        new PrecisionShotTable.Entry(%.2f, %.1f, %.2f)%s\n",
                    entry.distanceInches,
                    entry.targetRpm,
                    entry.hoodAngleDeg,
                    i == entries.size() - 1 ? "" : ","));
        }
        builder.append("));");
        return builder.toString();
    }

    public static PrecisionShotTable fromArray(double[][] rows) {
        ArrayList<Entry> entries = new ArrayList<>();
        for (double[] row : rows) {
            if (row.length >= 3) {
                entries.add(new Entry(row[0], row[1], row[2]));
            }
        }
        return new PrecisionShotTable(entries);
    }

    public static PrecisionShotTable defaultTable() {
        return PrecisionShotTable.fromArray(new double[][]{
                {36.0, 2650.0, 52.0},
                {48.0, 2800.0, 49.0},
                {60.0, 2950.0, 46.0},
                {72.0, 3080.0, 43.5},
                {84.0, 3220.0, 40.5},
                {96.0, 3375.0, 38.0},
                {108.0, 3525.0, 35.5},
                {120.0, 3680.0, 33.0}
        });
    }
}
