package org.firstinspires.ftc.teamcode.OpMode.TeleOp;


import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.LowPassFilter;
import com.pedropathing.geometry.Pose;

public class ConvertToPedroPose {
    public Pose convertToPedroPose(Pose filterPose) {
        if (filterPose == null) return null;

        double x = filterPose.getX();
        double y = filterPose.getY();
        double headingDegrees = filterPose.getHeading();

        // --- Convert to Pedro coordinate system ---
        double xPedro = y + 72;
        double yPedro = 72 - x;

        // Adjust heading for 90° CCW rotation
        double headingPedro = headingDegrees + 00.0;
        headingPedro = ((headingPedro + 360) % 360);  // normalize to [0,360)

        return new Pose(xPedro, yPedro, headingPedro);
}
}
