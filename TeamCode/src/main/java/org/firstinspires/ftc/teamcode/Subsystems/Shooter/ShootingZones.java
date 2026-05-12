package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class ShootingZones {

    public static double robotLeftMm = 218.35000;
    public static double robotRightMm = 218.35000;
    public static double robotFrontMm = 253.55376;
    public static double robotBackMm = 199.45376;
    public static double paddingInches = 0.0;
    public static double autoPaddingInches = 0.0;

    private static final double MM_PER_INCH = 25.4;

    private ShootingZones() {
    }

    public static boolean isInShootingZone(double x, double y) {
        return isInShootingZone(x, y, 0.0, paddingInches);
    }

    public static boolean isInShootingZone(double x, double y, double headingRadians) {
        return isInShootingZone(x, y, headingRadians, paddingInches);
    }

    public static boolean isInShootingZone(double x, double y, double headingRadians, double paddingInches) {
        return isInBackZone(x, y, headingRadians, paddingInches)
                || isInFrontZone(x, y, headingRadians, paddingInches);
    }

    public static boolean isInBackZone(double x, double y) {
        return isInBackZone(x, y, 0.0, paddingInches);
    }

    public static boolean isInBackZone(double x, double y, double headingRadians, double paddingInches) {
        return overlapsHalfPlane(x, y, headingRadians, paddingInches, -1.0, 1.0, -48.0)
                && overlapsHalfPlane(x, y, headingRadians, paddingInches, 1.0, 1.0, 96.0);
    }

    public static boolean isInFrontZone(double x, double y) {
        return isInFrontZone(x, y, 0.0, paddingInches);
    }

    public static boolean isInFrontZone(double x, double y, double headingRadians, double paddingInches) {
        return overlapsHalfPlane(x, y, headingRadians, paddingInches, -1.0, -1.0, -144.0)
                && overlapsHalfPlane(x, y, headingRadians, paddingInches, 1.0, -1.0, 0.0);
    }

    private static boolean overlapsHalfPlane(double x,
                                             double y,
                                             double headingRadians,
                                             double paddingInches,
                                             double normalX,
                                             double normalY,
                                             double limit) {
        double centerValue = normalX * x + normalY * y;
        double footprintReach = rectangleSupport(headingRadians, -normalX, -normalY)
                + Math.max(0.0, paddingInches) * Math.hypot(normalX, normalY);
        return centerValue <= limit + footprintReach;
    }

    private static double rectangleSupport(double headingRadians, double directionX, double directionY) {
        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);

        double forwardProjection = directionX * cos + directionY * sin;
        double leftProjection = directionX * -sin + directionY * cos;

        double forwardReach = forwardProjection >= 0.0
                ? inches(robotFrontMm) * forwardProjection
                : -inches(robotBackMm) * forwardProjection;
        double leftReach = leftProjection >= 0.0
                ? inches(robotLeftMm) * leftProjection
                : -inches(robotRightMm) * leftProjection;

        return forwardReach + leftReach;
    }

    private static double inches(double millimeters) {
        return millimeters / MM_PER_INCH;
    }
}
