package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.pedropathing.geometry.Pose;

public class ConvertToPedroPose {

    // Adjust ONLY if needed (start at 0)
    private static final double HEADING_OFFSET_DEG = 0.0;

    public static Pose convertToPedroPose(double xIn, double yIn, double headingDeg) {
        return new Pose(
                xIn,
                yIn,
                Math.toRadians(headingDeg + HEADING_OFFSET_DEG)
        );
    }
}
