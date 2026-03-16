package org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision;

final class ShooterMath {

    private ShooterMath() {}

    static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    static double lerp(double a, double b, double t) {
        return a + (b - a) * clamp(t, 0.0, 1.0);
    }

    static double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    static double solveLaunchSpeed(double horizontalDistanceInches,
                                   double verticalDeltaInches,
                                   double angleRad,
                                   double gravityIps2) {
        double cos = Math.cos(angleRad);
        double tan = Math.tan(angleRad);
        double denominator = 2.0 * cos * cos * (horizontalDistanceInches * tan - verticalDeltaInches);
        if (denominator <= 1e-6) {
            return Double.NaN;
        }
        double numerator = gravityIps2 * horizontalDistanceInches * horizontalDistanceInches;
        return Math.sqrt(numerator / denominator);
    }
}
