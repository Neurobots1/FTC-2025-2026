package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tunning;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class Motor extends OpMode {

    DcMotor motor;
    DcMotor shoot1;
    DcMotor shoot2;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        shoot1 = hardwareMap.get(DcMotor.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotor.class, "shoot2");

        // Optional: don't brake when power = 0
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        // 🔥 Run motor full power every loop
        motor.setPower(1.0);
        shoot1.setPower(0.5);
        shoot2.setPower(-0.5);
    }
}

