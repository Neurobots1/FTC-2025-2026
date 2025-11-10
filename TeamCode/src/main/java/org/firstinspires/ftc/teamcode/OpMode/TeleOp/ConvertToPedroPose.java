package org.firstinspires.ftc.teamcode.OpMode.TeleOp;


import com.pedropathing.geometry.Pose;

public class ConvertToPedroPose {
    public Pose convertToPedroPose(Pose ftcPose) {
        if (ftcPose == null) return null;

        double x = ftcPose.getX();
        double y = ftcPose.getY();
        double headingDegrees = ftcPose.getHeading();

        // --- Convert to Pedro coordinate system ---
        double xPedro = y + 72;
        double yPedro = 72 - x;

        // Adjust heading for 90° CCW rotation
        double headingPedro = headingDegrees + 00.0;
        headingPedro = ((headingPedro + 360) % 360);  // normalize to [0,360)

        return new Pose(xPedro, yPedro, headingPedro);
}
}
