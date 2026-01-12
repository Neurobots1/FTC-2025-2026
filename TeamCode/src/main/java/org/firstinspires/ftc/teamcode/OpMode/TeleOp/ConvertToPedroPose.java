package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.pedropathing.geometry.Pose;

public class ConvertToPedroPose {

    // Static utility method – no need to store fields
    public static Pose convertToPedroPose(Pose filterPose) {
        if (filterPose == null) return null;  // safety

        double x = filterPose.getX();
        double y = filterPose.getY();
        double headingRad = filterPose.getHeading();  // Pedro Pose usually uses radians

        // --- Your axis swap / translation (assuming FTC -> Pedro conversion) ---
        double xPedro = y + 72;
        double yPedro = 72 - x;

        // If heading is in radians, apply rotation in radians too.
        // If your filterPose heading is actually in DEGREES, convert first.
        double headingPedro = headingRad;  // adjust here if you rotate frame

        return new Pose(xPedro, yPedro, headingPedro);
    }
}
