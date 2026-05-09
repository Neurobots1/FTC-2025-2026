package org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular;

import com.pedropathing.geometry.Pose;

public final class AllianceMirroring {
    public static double FIELD_WIDTH_INCHES = 141.5;

    private AllianceMirroring() {}

    public static Pose forAlliance(Pose bluePose, AutoAlliance alliance) {
        if (alliance == AutoAlliance.BLUE) {
            return bluePose;
        }
        return mirrorXHeading(bluePose);
    }

    public static Pose mirrorXHeading(Pose pose) {
        return new Pose(
                FIELD_WIDTH_INCHES - pose.getX(),
                pose.getY(),
                normalizeRadians(Math.PI - pose.getHeading())
        );
    }

    public static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
