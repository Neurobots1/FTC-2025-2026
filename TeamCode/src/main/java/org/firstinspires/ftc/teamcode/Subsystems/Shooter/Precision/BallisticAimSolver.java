package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision;

final class BallisticAimSolver {

    static final class Solution {
        final boolean valid;
        final double hoodAngleRad;
        final double worldYawRad;
        final double flightTimeSeconds;
        final double launchSpeedIps;
        final double rangeInches;
        final String reason;

        Solution(boolean valid,
                 double hoodAngleRad,
                 double worldYawRad,
                 double flightTimeSeconds,
                 double launchSpeedIps,
                 double rangeInches,
                 String reason) {
            this.valid = valid;
            this.hoodAngleRad = hoodAngleRad;
            this.worldYawRad = worldYawRad;
            this.flightTimeSeconds = flightTimeSeconds;
            this.launchSpeedIps = launchSpeedIps;
            this.rangeInches = rangeInches;
            this.reason = reason;
        }

        static Solution invalid(String reason) {
            return new Solution(false, 0.0, 0.0, 0.0, 0.0, 0.0, reason);
        }
    }

    private BallisticAimSolver() {}

    static Solution solveWithFixedHood(double robotX,
                                       double robotY,
                                       double targetX,
                                       double targetY,
                                       double targetDeltaZ,
                                       double robotVx,
                                       double robotVy,
                                       double launchSpeedIps,
                                       double hoodAngleRad,
                                       double gravityIps2) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double range = Math.hypot(dx, dy);
        if (range < 1e-6 || launchSpeedIps <= 1e-6) {
            return Solution.invalid("target unreachable");
        }

        return buildSolution(dx, dy, robotVx, robotVy, targetDeltaZ, launchSpeedIps, hoodAngleRad, gravityIps2);
    }

    static Solution solve(double robotX,
                          double robotY,
                          double targetX,
                          double targetY,
                          double targetDeltaZ,
                          double robotVx,
                          double robotVy,
                          double launchSpeedIps,
                          double minHoodRad,
                          double maxHoodRad,
                          double gravityIps2) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double range = Math.hypot(dx, dy);
        if (range < 1e-6 || launchSpeedIps <= 1e-6) {
            return Solution.invalid("target unreachable");
        }

        double left = minHoodRad;
        double right = maxHoodRad;
        double fLeft = verticalResidual(dx, dy, targetDeltaZ, robotVx, robotVy, launchSpeedIps, left, gravityIps2);
        double fRight = verticalResidual(dx, dy, targetDeltaZ, robotVx, robotVy, launchSpeedIps, right, gravityIps2);

        if (Double.isNaN(fLeft) || Double.isNaN(fRight)) {
            return Solution.invalid("no ballistic bracket");
        }

        if (Math.signum(fLeft) == Math.signum(fRight)) {
            double bestAngle = minHoodRad;
            double bestResidual = Double.POSITIVE_INFINITY;
            for (int i = 0; i <= 48; i++) {
                double angle = minHoodRad + (maxHoodRad - minHoodRad) * (i / 48.0);
                double residual = verticalResidual(dx, dy, targetDeltaZ, robotVx, robotVy, launchSpeedIps, angle, gravityIps2);
                if (!Double.isNaN(residual) && Math.abs(residual) < bestResidual) {
                    bestResidual = Math.abs(residual);
                    bestAngle = angle;
                }
            }
            if (bestResidual > 1.0) {
                return Solution.invalid("target unreachable");
            }
            return buildSolution(dx, dy, robotVx, robotVy, targetDeltaZ, launchSpeedIps, bestAngle, gravityIps2);
        }

        for (int i = 0; i < 28; i++) {
            double mid = 0.5 * (left + right);
            double fMid = verticalResidual(dx, dy, targetDeltaZ, robotVx, robotVy, launchSpeedIps, mid, gravityIps2);
            if (Double.isNaN(fMid)) {
                return Solution.invalid("invalid trajectory");
            }
            if (Math.abs(fMid) < 1e-3) {
                return buildSolution(dx, dy, robotVx, robotVy, targetDeltaZ, launchSpeedIps, mid, gravityIps2);
            }
            if (Math.signum(fMid) == Math.signum(fLeft)) {
                left = mid;
                fLeft = fMid;
            } else {
                right = mid;
                fRight = fMid;
            }
        }

        return buildSolution(dx, dy, robotVx, robotVy, targetDeltaZ, launchSpeedIps, 0.5 * (left + right), gravityIps2);
    }

    private static Solution buildSolution(double dx,
                                          double dy,
                                          double robotVx,
                                          double robotVy,
                                          double targetDeltaZ,
                                          double launchSpeedIps,
                                          double hoodAngleRad,
                                          double gravityIps2) {
        double flightTime = horizontalFlightTime(dx, dy, robotVx, robotVy, launchSpeedIps, hoodAngleRad);
        if (Double.isNaN(flightTime) || flightTime <= 0.0) {
            return Solution.invalid("bad flight time");
        }

        double reqVx = dx / flightTime - robotVx;
        double reqVy = dy / flightTime - robotVy;
        double worldYaw = Math.atan2(reqVy, reqVx);
        double vertical = launchSpeedIps * Math.sin(hoodAngleRad) * flightTime
                - 0.5 * gravityIps2 * flightTime * flightTime;
        if (Math.abs(vertical - targetDeltaZ) > 2.0) {
            return Solution.invalid("vertical miss");
        }

        return new Solution(true, hoodAngleRad, worldYaw, flightTime, launchSpeedIps, Math.hypot(dx, dy), "ok");
    }

    private static double verticalResidual(double dx,
                                           double dy,
                                           double targetDeltaZ,
                                           double robotVx,
                                           double robotVy,
                                           double launchSpeedIps,
                                           double hoodAngleRad,
                                           double gravityIps2) {
        double flightTime = horizontalFlightTime(dx, dy, robotVx, robotVy, launchSpeedIps, hoodAngleRad);
        if (Double.isNaN(flightTime) || flightTime <= 0.0) {
            return Double.NaN;
        }
        double vertical = launchSpeedIps * Math.sin(hoodAngleRad) * flightTime
                - 0.5 * gravityIps2 * flightTime * flightTime;
        return vertical - targetDeltaZ;
    }

    private static double horizontalFlightTime(double dx,
                                               double dy,
                                               double robotVx,
                                               double robotVy,
                                               double launchSpeedIps,
                                               double hoodAngleRad) {
        double projectileHorizontalSpeed = launchSpeedIps * Math.cos(hoodAngleRad);
        if (projectileHorizontalSpeed <= 1e-6) {
            return Double.NaN;
        }

        double a = robotVx * robotVx + robotVy * robotVy - projectileHorizontalSpeed * projectileHorizontalSpeed;
        double b = -2.0 * (dx * robotVx + dy * robotVy);
        double c = dx * dx + dy * dy;

        if (Math.abs(a) < 1e-6) {
            if (Math.abs(b) < 1e-6) {
                return Double.NaN;
            }
            double t = -c / b;
            return t > 0.0 ? t : Double.NaN;
        }

        double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) {
            return Double.NaN;
        }

        double sqrt = Math.sqrt(discriminant);
        double t1 = (-b - sqrt) / (2.0 * a);
        double t2 = (-b + sqrt) / (2.0 * a);
        double best = Double.NaN;
        if (t1 > 0.0) {
            best = t1;
        }
        if (t2 > 0.0 && (Double.isNaN(best) || t2 < best)) {
            best = t2;
        }
        return best;
    }
}
