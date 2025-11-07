package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Shooter Spin Test", group = "Test")
public class ShooterSpinTest extends OpMode {

    private DcMotorEx shootMotor1;
    private DcMotorEx shootMotor2;

    @Override
    public void init() {
        // Initialize motors from hardware map
        shootMotor1 = hardwareMap.get(DcMotorEx.class, "shootMotor1");
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");

        // Set motor directions
        shootMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Shooter motors initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Spin both motors at full power
        shootMotor1.setPower(1.0);
        shootMotor2.setPower(1.0);

        // Optional telemetry
        telemetry.addData("ShootMotor1 Power", shootMotor1.getPower());
        telemetry.addData("ShootMotor2 Power", shootMotor2.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop motors when the OpMode ends
        shootMotor1.setPower(0);
        shootMotor2.setPower(0);
    }
}
