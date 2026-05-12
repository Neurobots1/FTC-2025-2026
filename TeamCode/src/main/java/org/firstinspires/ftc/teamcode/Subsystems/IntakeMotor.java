package org.firstinspires.ftc.teamcode.Subsystems;



import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.HardwareMapConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

public class IntakeMotor {
    private DcMotorEx intakeMotor;

    public IntakeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, HardwareMapConstants.INTAKE_MOTOR);

    }

    public void intake() {
        intakeMotor.setPower(-1);
    }

    public void preFeedIntake() {
        intakeMotor.setPower(ShooterConstants.preFeedIntakePower);
    }

    public void slowIntake() {
        intakeMotor.setPower(ShooterConstants.mainFeedIntakePower);
    }

    public void outtake() {
        intakeMotor.setPower(0.7);  // Run motor at 3/4 speed in reverse for outtake
    }

    // Method to run the motor at half speed to keep intaked pieces inside
    public void slowOuttake() {
        intakeMotor.setPower(0.3);  // Run motor at half speed to keep pieces inside
    }

    // Method to stop the intake motor
    public void stop() {
        intakeMotor.setPower(0);  // Stop motor
    }
}
