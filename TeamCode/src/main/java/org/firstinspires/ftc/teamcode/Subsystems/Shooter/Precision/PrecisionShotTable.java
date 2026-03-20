package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

public final class PrecisionShotTable {

    private static final int ROW_DISTANCE_INCHES = 0;
    private static final int ROW_TARGET_RPM = 1;
    private static final int ROW_HOOD_ANGLE_DEG = 2;

    public static final class Entry {
        // Horizontal distance from the shooter/turret pivot to the goal center.
        public final double distanceInches;
        // Flywheel speed target for this distance.
        public final double targetRpm;
        // Hood launch angle in degrees for this distance.
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
                .append(" = new PrecisionShotTable(Arrays.asList(\n")
                .append("        // Entry(distanceInches, targetRpm, hoodAngleDeg)\n");
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
                entries.add(new Entry(
                        row[ROW_DISTANCE_INCHES],
                        row[ROW_TARGET_RPM],
                        row[ROW_HOOD_ANGLE_DEG]
                ));
            }
        }
        return new PrecisionShotTable(entries);
    }

    public static PrecisionShotTable defaultTable() {
        return new PrecisionShotTable(Arrays.asList(
                // Entry(distanceInches, targetRpm, hoodAngleDeg)
                new Entry(36.0, 2650.0, 52.0),
                new Entry(48.0, 2800.0, 49.0),
                new Entry(60.0, 2950.0, 46.0),
                new Entry(72.0, 3080.0, 43.5),
                new Entry(84.0, 3220.0, 40.5),
                new Entry(96.0, 3375.0, 38.0),
                new Entry(108.0, 3525.0, 35.5),
                new Entry(120.0, 3680.0, 33.0)
        ));
    }
}
