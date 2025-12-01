package org.firstinspires.ftc.teamcode.SubSystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Shoot {

    public DcMotorEx ShooterA;
    public DcMotorEx ShooterB;

    private PIDController controller1;
    private double targetRPM = 0;

    private double currentRPM1 = 0;
    private double currentRPM2 = 0;

            public static double p= 0;
            public static double i = 0;
            public static double d= 0;
            public static  double f=0;

    private static final double TICKS_PER_REV = 28.0; // adjust if needed


    public Shoot(HardwareMap hardwareMap) {
        ShooterA = hardwareMap.get(DcMotorEx.class, "ShooterA");
        ShooterA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ShooterA.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterB = hardwareMap.get(DcMotorEx.class, "ShooterB");
        ShooterB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ShooterB.setDirection(DcMotorSimple.Direction.FORWARD);

        controller1 = new PIDController(p, i, d);
    }


        public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

        public void update() {

            controller1 = new PIDController(p, i, d);
            // Convert ticks/s to RPM
            currentRPM1 = ShooterA.getVelocity() * 60.0 / TICKS_PER_REV;
            currentRPM2 = ShooterB.getVelocity() * 60.0 / TICKS_PER_REV;

            double currentRPM = (currentRPM1 + currentRPM2) / 2.0;

            double power = controller1.calculate(targetRPM, currentRPM);

            power = Math.max(0, Math.min(1, power));

            ShooterA.setPower(power);
            ShooterB.setPower(power);
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
