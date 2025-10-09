package org.firstinspires.ftc.teamcode.SubSystem.component;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorDataLogger extends OpMode {

    private ColorSensor colorSensor;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        telemetry.addData("Status", "Color Sensor Initialized");
    }

    @Override
    public void loop() {
        // Get the current RGB values from the color sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Display the raw RGB values on the telemetry screen
        telemetry.addData("Raw RGB Values", "R=%d, G=%d, B=%d", red, green, blue);

        // Update the telemetry
        telemetry.update();
    }
}
