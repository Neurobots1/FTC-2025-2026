package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shoot {

    public DcMotorEx shootMotor1;
    public DcMotorEx shootMotor2;

    public Servo hoodServo;
    public Servo OnOff;

    private PIDController controller1;
    private PIDController controller2;
    private double targetRPM = 0;

    public Shoot(HardwareMap hardwareMap, double p1, double i1, double d1,
                 double p2, double i2, double d2) {
        shootMotor1 = hardwareMap.get(DcMotorEx.class, "shootMotor1");
        shootMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");
        shootMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        hoodServo = hardwareMap.get(Servo.class, "HoodServo");

        controller1 = new PIDController(p1, i1, d1);
        controller2 = new PIDController(p2,i2,d2);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    public void update() {
        // Get current RPMs (ticks per second to RPM if needed)
        double motor1RPM = shootMotor1.getVelocity() * 60.0 / 28.0; // Example: if motor has 28 ticks/rev
        double motor2RPM = shootMotor2.getVelocity() * 60.0 / 28.0;

        // Average both motors
        double currentRPM = (motor1RPM + motor2RPM) / 2.0;

        // Calculate power correction
        // PID control for each motor separately
        double power1 = controller1.calculate(targetRPM, motor1RPM);
        double power2 = controller2.calculate(targetRPM, motor2RPM);
        // Apply power limit (to stay safe)

        shootMotor1.setPower(power1);
        shootMotor2.setPower(power2);
    }


    public void On(){
        hoodServo.setPosition(1);
    }

    public void Off(){
        hoodServo.setPosition(0);
    }
}
