package org.firstinspires.ftc.teamcode.SubSystem.Vision;

import com.pedropathing.geometry.Pose;

public class ConvertToPedroPose {

    public static Pose convertToPedroPose(double xCenterIn, double yCenterIn, double headingDeg) {
        double xRot = -yCenterIn;
        double yRot =  xCenterIn;

        double xPedro = xRot + 72.0;
        double yPedro = yRot + 72.0;

        double headingPedroRad = Math.toRadians(headingDeg + 90.0);

        return new Pose(xPedro, yPedro, headingPedroRad);
    }
}
