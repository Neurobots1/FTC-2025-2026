package org.firstinspires.ftc.teamcode.SubSystem;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMotor {
    private DcMotorEx intakeMotor;

    public IntakeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

    }

    public void intake() {
        intakeMotor.setPower(1);
    }

    public void slowIntake() {
        intakeMotor.setPower(0.7);
    }

    public void SlowIntk(){
        intakeMotor.setPower(-0.7);
    }


    public void outtake() {
        intakeMotor.setPower(-0.7);  // Run motor at 3/4 speed in reverse for outtake
    }

    // Method to run the motor at half speed to keep intaked pieces inside
    public void slowOuttake() {
        intakeMotor.setPower(-0.3);  // Run motor at half speed to keep pieces inside
    }

    // Method to stop the intake motor
    public void stop() {
        intakeMotor.setPower(0);  // Stop motor
    }
}