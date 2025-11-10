package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.SubSystem.Servo_HOOD;

@Disabled

@Autonomous(name = "Test_Servo_Hood", group = "Examples")
public class Test_Servo extends OpMode {

    private Servo_HOOD servoHood;
    private double minDegree = 0;
    private double maxDegree = 36;
    private double currentAngle = 0;
    private boolean increasing = true;
    private long lastUpdateTime = 0;
    private long delay = 500;



    public void setAngle(double degrees) {
        // On garde l’angle dans la plage
        degrees = Math.max(minDegree, Math.min(maxDegree, degrees));
        double position = degreesToServoPosition(degrees);

        // Sécurité : forcer entre 0 et 1
        position = Math.max(0.0, Math.min(1.0, position));

        // Envoi réel au servo
        servoHood.setAngle(position);

    }


    public double degreesToServoPosition(double degrees) {
        return (degrees - minDegree) / (maxDegree - minDegree);
    }
    public void setServoPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        servoHood.setAngle(position);
    }

    public double getCurrentAngle() {
        double position = servoHood.getCurrentAngle();
        return minDegree + position * (maxDegree - minDegree);
    }





    @Override
    public void start() {
        currentAngle = minDegree;
        setAngle(currentAngle);

    }


    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();

        if (currentTime - lastUpdateTime > delay) {
            if (increasing) {
                currentAngle += 5; // monte de 5°
                if (currentAngle >= maxDegree) {
                    currentAngle = maxDegree;
                    increasing = false;
                }
            } else {
                currentAngle -= 5; // descend de 5°
                if (currentAngle <= minDegree) {
                    currentAngle = minDegree;
                    increasing = true;
                }
            }
            setAngle(currentAngle);
            lastUpdateTime = currentTime;
        }

        telemetry.addData("Servo Angle", currentAngle);
        telemetry.addData("Servo Position", servoHood.getCurrentAngle());
        telemetry.update();
    }

            @Override
            public void init() {
                servoHood = new Servo_HOOD(hardwareMap, "hoodServo");
                telemetry.addLine("Servo Hood prêt !");

            }
}


