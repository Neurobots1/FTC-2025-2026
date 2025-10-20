package org.firstinspires.ftc.teamcode.SubSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shoot {
    public Robot robot;

    public DcMotorEx shootMotor1;
    public DcMotorEx shootMotor2;


    public Shoot(HardwareMap hardwareMap) {
        //futurShootMotor
        shootMotor1 = hardwareMap.get(DcMotorEx.class, "shootMotor1");
        shootMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");
        shootMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void ShootM() {
        shootMotor1.setPower(-1.0);  // Run motor at full speed forward for Shoot
        shootMotor2.setPower(-1);
    }


    public void stop() {
        shootMotor1.setPower(0);  // Stop shooting
        shootMotor2.setPower(0);
    }
}

