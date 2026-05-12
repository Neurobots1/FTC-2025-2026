package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision;

final class ShootOnMoveCalculator {

    static final class Solution {
        final boolean valid;
        final double hoodAngleRad;
        final double worldYawRad;
        final double flightTimeSeconds;
        final double launchSpeedIps;
        final double rangeInches;
        final double stationaryHoodAngleRad;
        final double stationaryLaunchSpeedIps;
        final String reason;

        Solution(boolean valid,
                 double hoodAngleRad,
                 double worldYawRad,
                 double flightTimeSeconds,
                 double launchSpeedIps,
                 double rangeInches,
                 double stationaryHoodAngleRad,
                 double stationaryLaunchSpeedIps,
                 String reason) {
            this.valid = valid;
            this.hoodAngleRad = hoodAngleRad;
            this.worldYawRad = worldYawRad;
            this.flightTimeSeconds = flightTimeSeconds;
            this.launchSpeedIps = launchSpeedIps;
            this.rangeInches = rangeInches;
            this.stationaryHoodAngleRad = stationaryHoodAngleRad;
            this.stationaryLaunchSpeedIps = stationaryLaunchSpeedIps;
            this.reason = reason;
        }

        static Solution invalid(String reason) {
            return new Solution(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, reason);
        }
    }

    private static final class StationaryShot {
        final double hoodAngleRad;
        final double launchSpeedIps;
        final double flightTimeSeconds;

        StationaryShot(double hoodAngleRad, double launchSpeedIps, double flightTimeSeconds) {
            this.hoodAngleRad = hoodAngleRad;
            this.launchSpeedIps = launchSpeedIps;
            this.flightTimeSeconds = flightTimeSeconds;
        }
    }

    private ShootOnMoveCalculator() {}

    static Solution solveStationary(double shooterX,
                                    double shooterY,
                                    double targetX,
                                    double targetY,
                                    double targetDeltaZ,
                                    double goalEntryAngleRad,
                                    double gravityIps2) {
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;
        double range = Math.hypot(dx, dy);
        if (range <= 1e-6) {
            return Solution.invalid("target unreachable");
        }

        StationaryShot stationaryShot = solveStationaryShot(range, targetDeltaZ, goalEntryAngleRad, gravityIps2);
        if (stationaryShot == null) {
            return Solution.invalid("stationary shot");
        }

        return new Solution(
                true,
                stationaryShot.hoodAngleRad,
                Math.atan2(dy, dx),
                stationaryShot.flightTimeSeconds,
                stationaryShot.launchSpeedIps,
                range,
                stationaryShot.hoodAngleRad,
                stationaryShot.launchSpeedIps,
                "stationary"
        );
    }

    static Solution solveCompensated(double shooterX,
                                     double shooterY,
                                     double targetX,
                                     double targetY,
                                     double targetDeltaZ,
                                     double shooterVx,
                                     double shooterVy,
                                     double goalEntryAngleRad,
                                     double minHoodRad,
                                     double maxHoodRad,
                                     double gravityIps2) {
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;
        double range = Math.hypot(dx, dy);
        if (range <= 1e-6) {
            return Solution.invalid("target unreachable");
        }

        StationaryShot stationaryShot = solveStationaryShot(range, targetDeltaZ, goalEntryAngleRad, gravityIps2);
        if (stationaryShot == null) {
            return Solution.invalid("stationary shot");
        }

        double unitRadialX = dx / range;
        double unitRadialY = dy / range;
        double unitTangentialX = -unitRadialY;
        double unitTangentialY = unitRadialX;

        double radialVelocity = shooterVx * unitRadialX + shooterVy * unitRadialY;
        double tangentialVelocity = shooterVx * unitTangentialX + shooterVy * unitTangentialY;
        double stationaryHorizontalVelocity = stationaryShot.launchSpeedIps * Math.cos(stationaryShot.hoodAngleRad);
        double stationaryVerticalVelocity = stationaryShot.launchSpeedIps * Math.sin(stationaryShot.hoodAngleRad);

        if (stationaryHorizontalVelocity <= 1e-6) {
            return Solution.invalid("horizontal speed");
        }

        double compensatedRadialVelocity = range / stationaryShot.flightTimeSeconds - radialVelocity;
        if (compensatedRadialVelocity <= 1e-6) {
            return Solution.invalid("radial compensation");
        }

        double compensatedHorizontalVelocity = Math.hypot(compensatedRadialVelocity, tangentialVelocity);
        double unclampedHoodAngleRad = Math.atan2(stationaryVerticalVelocity, compensatedHorizontalVelocity);
        double hoodAngleRad = ShooterMath.clamp(unclampedHoodAngleRad, minHoodRad, maxHoodRad);
        double compensatedHorizontalDistance = compensatedHorizontalVelocity * stationaryShot.flightTimeSeconds;
        double launchSpeedIps = ShooterMath.solveLaunchSpeed(
                compensatedHorizontalDistance,
                targetDeltaZ,
                hoodAngleRad,
                gravityIps2
        );
        if (Double.isNaN(launchSpeedIps) || launchSpeedIps <= 1e-6) {
            return Solution.invalid("launch speed");
        }

        double launchHorizontalVelocity = launchSpeedIps * Math.cos(hoodAngleRad);
        if (launchHorizontalVelocity <= 1e-6) {
            return Solution.invalid("clamped hood");
        }

        double flightTimeSeconds = compensatedHorizontalDistance / launchHorizontalVelocity;
        double aimX = compensatedRadialVelocity * unitRadialX - tangentialVelocity * unitTangentialX;
        double aimY = compensatedRadialVelocity * unitRadialY - tangentialVelocity * unitTangentialY;

        return new Solution(
                true,
                hoodAngleRad,
                Math.atan2(aimY, aimX),
                flightTimeSeconds,
                launchSpeedIps,
                range,
                stationaryShot.hoodAngleRad,
                stationaryShot.launchSpeedIps,
                "shoot on move"
        );
    }

    private static StationaryShot solveStationaryShot(double horizontalDistanceInches,
                                                      double targetDeltaZ,
                                                      double goalEntryAngleRad,
                                                      double gravityIps2) {
        if (horizontalDistanceInches <= 1e-6) {
            return null;
        }

        double tanLaunchAngle = (2.0 * targetDeltaZ / horizontalDistanceInches) - Math.tan(goalEntryAngleRad);
        double hoodAngleRad = Math.atan(tanLaunchAngle);
        double launchSpeedIps = ShooterMath.solveLaunchSpeed(
                horizontalDistanceInches,
                targetDeltaZ,
                hoodAngleRad,
                gravityIps2
        );
        if (Double.isNaN(launchSpeedIps) || launchSpeedIps <= 1e-6) {
            return null;
        }

        double horizontalVelocity = launchSpeedIps * Math.cos(hoodAngleRad);
        if (horizontalVelocity <= 1e-6) {
            return null;
        }

        return new StationaryShot(
                hoodAngleRad,
                launchSpeedIps,
                horizontalDistanceInches / horizontalVelocity
        );
    }
}
