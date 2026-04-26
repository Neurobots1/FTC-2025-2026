package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShotTable;

@Configurable
public final class LUTConstants {

    private LUTConstants() {
    }

    // Panels can tune primitive fields live, so the shot table is exposed as rows instead
    // of a single complex object.
    public static double shot1DistanceInches = 34.2417;
    public static double shot1TargetRpm = 2525.0;
    public static double shot1HoodAngleDeg = 30.00;

    public static double shot2DistanceInches = 58.9609;
    public static double shot2TargetRpm = 2800.0;
    public static double shot2HoodAngleDeg = 38.00;

    public static double shot3DistanceInches = 75.9841;
    public static double shot3TargetRpm = 3025.00;
    public static double shot3HoodAngleDeg = 40.25;

    public static double shot4DistanceInches = 91.4179;
    public static double shot4TargetRpm = 3225.0;
    public static double shot4HoodAngleDeg = 43.25;

    public static double shot5DistanceInches = 107.1952;
    public static double shot5TargetRpm = 3475.0;
    public static double shot5HoodAngleDeg = 46.5;

    public static double shot6DistanceInches = 121.2883;
    public static double shot6TargetRpm = 3700.0;
    public static double shot6HoodAngleDeg = 49.75;

    public static double shot7DistanceInches = 139.67;
    public static double shot7TargetRpm = 4325;
    public static double shot7HoodAngleDeg = 55.00;

    public static double shot8DistanceInches = 154.7216;
    public static double shot8TargetRpm = 4450;
    public static double shot8HoodAngleDeg = 55.0;

    public static double shot9DistanceInches = 0.0;
    public static double shot9TargetRpm = 0.0;
    public static double shot9HoodAngleDeg = 0.0;

    public static double shot10DistanceInches = 0.0;
    public static double shot10TargetRpm = 0.0;
    public static double shot10HoodAngleDeg = 0.0;

    public static double shot11DistanceInches = 0.0;
    public static double shot11TargetRpm = 0.0;
    public static double shot11HoodAngleDeg = 0.0;

    public static double shot12DistanceInches = 0.0;
    public static double shot12TargetRpm = 0.0;
    public static double shot12HoodAngleDeg = 0.0;

    public static PrecisionShotTable currentTable() {
        return PrecisionShotTable.fromArray(new double[][]{
                {shot1DistanceInches, shot1TargetRpm, shot1HoodAngleDeg},
                {shot2DistanceInches, shot2TargetRpm, shot2HoodAngleDeg},
                {shot3DistanceInches, shot3TargetRpm, shot3HoodAngleDeg},
                {shot4DistanceInches, shot4TargetRpm, shot4HoodAngleDeg},
                {shot5DistanceInches, shot5TargetRpm, shot5HoodAngleDeg},
                {shot6DistanceInches, shot6TargetRpm, shot6HoodAngleDeg},
                {shot7DistanceInches, shot7TargetRpm, shot7HoodAngleDeg},
                {shot8DistanceInches, shot8TargetRpm, shot8HoodAngleDeg},
                {shot9DistanceInches, shot9TargetRpm, shot9HoodAngleDeg},
                {shot10DistanceInches, shot10TargetRpm, shot10HoodAngleDeg},
                {shot11DistanceInches, shot11TargetRpm, shot11HoodAngleDeg},
                {shot12DistanceInches, shot12TargetRpm, shot12HoodAngleDeg}
        });
    }
}
