package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.pedropathing.geometry.Pose;

public class ConvertToPedroPose {

    public static Pose convertToPedroPose(Pose centerPoseDeg) {
        if (centerPoseDeg == null) return null;

        double x = centerPoseDeg.getX();          // center-origin inches
        double y = centerPoseDeg.getY();          // center-origin inches
        double headingDeg = centerPoseDeg.getHeading(); // DEGREES in your FTC frame

        // Rotate position 90° left (CCW): (x, y) -> (-y, x)
        double xRot = -y;
        double yRot =  x;

        // Shift to Pedro field (72,72 is center)
        double xPedro = xRot + 72.0;
        double yPedro = yRot + 72.0;

        // Rotate heading with the same frame rotation, then convert to radians for Pedro
        double headingPedroRad = Math.toRadians(headingDeg + 90.0);

        return new Pose(xPedro, yPedro, headingPedroRad);
    }
}
