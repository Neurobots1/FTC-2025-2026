package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDFController;
// import com.seattlesolvers.solverslib.util.InterpLUT;  // <-- removed

@Configurable
public class Launcher23511 {

    public static double P = 0.03;
    public static double I = 0.003;
    public static double D = 0.00004;
    public static double F = 0.000039;

    public static double VELOCITY_TOLERANCE = 40;
    public static double MAX_FLYWHEEL_VELOCITY = 1860;
    public static double DEFAULT_ON_POWER = 0.75;

    public static double NOMINAL_VOLTAGE = 12.82;

    public static boolean MOTOR_ONE_REVERSED = false;
    public static boolean MOTOR_TWO_REVERSED = true;

    private final DcMotorEx flywheelMotorOne;
    private final DcMotorEx flywheelMotorTwo;
    private final VoltageSensor voltageSensor;
    private final PIDFController flywheelController;

    private boolean activeControl = false;
    private double targetVelocityInput = 0.0;

    // private final InterpLUT lut = new InterpLUT();  // <-- removed

    public Launcher23511(DcMotorEx flywheelMotorOne, DcMotorEx flywheelMotorTwo, VoltageSensor voltageSensor) {
        this.flywheelMotorOne = flywheelMotorOne;
        this.flywheelMotorTwo = flywheelMotorTwo;
        this.voltageSensor = voltageSensor;

        flywheelMotorOne.setDirection(MOTOR_ONE_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        flywheelMotorTwo.setDirection(MOTOR_TWO_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);

        flywheelController = new PIDFController(P, I, D, F);
        flywheelController.setTolerance(VELOCITY_TOLERANCE);

        // All LUT setup removed:
        // lut.add(...);
        // lut.createLUT();
    }

    public static Launcher23511 create(HardwareMap hardwareMap) {
        DcMotorEx flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        DcMotorEx flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        Launcher23511 launcher = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        launcher.init();
        return launcher;
    }

    public void init() {
        setFlywheel(0, false);
    }

    // velInput is now interpreted directly as ticksPerSecond
    public void setFlywheel(double velInput, boolean usePIDF) {
        targetVelocityInput = velInput;
        double ticksPerSecond = velInput;  // <-- direct, no LUT
        if (ticksPerSecond > MAX_FLYWHEEL_VELOCITY) ticksPerSecond = MAX_FLYWHEEL_VELOCITY;
        flywheelController.setSetPoint(ticksPerSecond);
        activeControl = usePIDF;
    }

    public void setFlywheelTicks(double ticksPerSecond) {
        flywheelController.setSetPoint(ticksPerSecond);
        activeControl = true;
    }

    public void update() {
        // If target is 0, don't control anything, just make sure we're off
        if (flywheelController.getSetPoint() == 0 || targetVelocityInput == 0) {
            activeControl = false;
            flywheelMotorOne.setPower(0);
            flywheelMotorTwo.setPower(0);
            return;
        }

        double voltage = voltageSensor.getVoltage();
        double normalizedVoltage = voltage / NOMINAL_VOLTAGE;
        if (normalizedVoltage <= 0) normalizedVoltage = 1.0;

        flywheelController.setPIDF(P, I, D, F / normalizedVoltage);

        flywheelMotorOne.setDirection(MOTOR_ONE_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        flywheelMotorTwo.setDirection(MOTOR_TWO_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);

        if (activeControl) {
            double currentVelocity = flywheelMotorOne.getVelocity();
            double power = flywheelController.calculate(currentVelocity);
            flywheelMotorOne.setPower(power);
            flywheelMotorTwo.setPower(power);
        } else {
            if (flywheelController.getSetPoint() == 0) {
                flywheelMotorOne.setPower(0);
                flywheelMotorTwo.setPower(0);
            } else {
                flywheelMotorOne.setPower(DEFAULT_ON_POWER);
                flywheelMotorTwo.setPower(DEFAULT_ON_POWER);
            }
        }
    }

    public void stop() {
        setFlywheelTicks(0);
        flywheelMotorOne.setPower(0);
        flywheelMotorTwo.setPower(0);
    }

    public boolean flywheelReady() {
        if (!activeControl) return false;
        if (flywheelMotorOne.getVelocity() <= VELOCITY_TOLERANCE) {
            return false;
        }

        double targetTPS = flywheelController.getSetPoint();
        double currentTPS = flywheelMotorOne.getVelocity();

        return Math.abs(targetTPS - currentTPS) <= VELOCITY_TOLERANCE;
    }
}
