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

    private double currentRPM1 = 0;
    private double currentRPM2 = 0;

    private static final double TICKS_PER_REV = 28.0; // adjust if needed


    public Shoot(HardwareMap hardwareMap, double p, double i, double d) {
        shootMotor1 = hardwareMap.get(DcMotorEx.class, "shootMotor1");
        shootMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");
        shootMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        controller1 = new PIDController(p, i, d);
    }


        public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

        public void update() {
            // Convert ticks/s to RPM
            currentRPM1 = shootMotor1.getVelocity() * 60.0 / TICKS_PER_REV;
            currentRPM2 = shootMotor2.getVelocity() * 60.0 / TICKS_PER_REV;

            double currentRPM = (currentRPM1 + currentRPM2) / 2.0;

            double power = controller1.calculate(targetRPM, currentRPM);

            power = Math.max(0, Math.min(1, power));

            shootMotor1.setPower(power);
            shootMotor2.setPower(power);
        }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM1(){
    return currentRPM1;
    }

    public double getAverageRPM() {
        return (currentRPM1 + currentRPM2) / 2.0;
    }
}
