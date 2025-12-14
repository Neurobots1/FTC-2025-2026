package org.firstinspires.ftc.teamcode.SubSystem;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMotor {
    private DcMotorEx intakeMotor;

    public IntakeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set the motor direction to FORWARD or REVERSE based on your setup

    }

    // Method to run the motor for intaking at full speed (positive power)
    public void intake() {
        intakeMotor.setPower(-1.0);  // Run motor at full speed forward for intake
    }

    // Method to run the motor for outtaking at half speed (negative power)
    public void outtake() {
        intakeMotor.setPower(1);  // Run motor at 3/4 speed in reverse for outtake
    }

    // Method to run the motor at half speed to keep intaked pieces inside
    public void slowOuttake() {
        intakeMotor.setPower(0.3);  // Run motor at half speed to keep pieces inside
    }

    // Method to stop the intake motor
    public void stop() {
        intakeMotor.setPower(0);  // Stop motor
    }

    public void getPower(){
       intakeMotor.getPower();
    }
}