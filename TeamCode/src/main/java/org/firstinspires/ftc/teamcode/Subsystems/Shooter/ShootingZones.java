package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class ShootingZones {

    public static double radiusInches = 10.0;

    private ShootingZones() {
    }

    public static boolean isInShootingZone(double x, double y) {
        return isInBackZone(x, y) || isInFrontZone(x, y);
    }

    public static boolean isInBackZone(double x, double y) {
        double d = radiusInches * 1.41421356237;
        return y <= x - 48 + d && y <= -x + 96 + d;
    }

    public static boolean isInFrontZone(double x, double y) {
        double d = radiusInches * 1.41421356237;
        return y >= -x + 144 - d && y >= x - d;
    }
}
