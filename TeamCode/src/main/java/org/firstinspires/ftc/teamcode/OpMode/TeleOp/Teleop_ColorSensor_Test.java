package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Teleop_ColorSensor_Test", group="TeleOp")
public class Teleop_ColorSensor_Test extends OpMode {
    private RevColorSensorV3 colorSensor;
    private IntakeMotor intkM;

    @Override
    public void init() {
        // Initialize the color sensor
        intkM = new IntakeMotor(hardwareMap);
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) intkM.intake();
        else if (gamepad1.left_bumper) intkM.outtake();
        else intkM.stop();
        // Get the distance from the color sensor
        double distance = colorSensor.getDistance(DistanceUnit.MM);

        // Read color sensor values
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];

        // Convert RGB to HSV
        Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );

        // Print hue value and distance
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Distance (mm)", distance);
        telemetry.update();
    }
}