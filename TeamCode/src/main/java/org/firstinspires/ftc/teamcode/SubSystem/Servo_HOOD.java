package org.firstinspires.ftc.teamcode.SubSystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servo_HOOD  {

    private Servo servoHood;
    private double minDegree;
    private double maxDegree;
    public Servo_HOOD(HardwareMap hardwareMap, String servoName) {
        servoHood = hardwareMap.get(Servo.class, servoName);
    }

    public Servo_HOOD(Servo servo, double minDegree, double maxDegree) {
        this.servoHood = servo;

        this.minDegree = 0;
        this.maxDegree = 36;
        
    }


    public void setAngle(double degrees) {
        degrees = Math.max(minDegree, Math.min(maxDegree, degrees));
        double position = degreesToServoPosition(degrees);
        servoHood.setPosition(position);
    }

    public double degreesToServoPosition(double degrees) {
        return (degrees - minDegree) / (maxDegree - minDegree);
    }

    public double getCurrentAngle() {
        double position = servoHood.getPosition();
        return minDegree + position * (maxDegree - minDegree);
    }
}