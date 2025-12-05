package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Configurable
@TeleOp(name = "ShooterTune23511_NoLUT", group = "Tuning")
public class ShooterTune23511 extends OpMode {

    public static boolean usePIDF = true;
    public static boolean shooterEnabled = false;
    public static boolean rawPowerMode = false;

    public static double targetTicksPerSecond = 1500.0;
    public static double testPower = 1.0;

    private Launcher23511 launcher;
    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;
    private VoltageSensor voltageSensor;
    private TelemetryManager telemetryManager;

    @Override
    public void init() {
        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "flywheel2");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcher = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        launcher.init();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad1.a) shooterEnabled = true;
        if (gamepad1.b) shooterEnabled = false;
        if (gamepad1.x) rawPowerMode = true;
        if (gamepad1.y) rawPowerMode = false;

        if (rawPowerMode) {
            flywheelMotorOne.setPower(testPower);
            flywheelMotorTwo.setPower(testPower);
        } else {
            if (shooterEnabled) {
                launcher.setFlywheelTicks(targetTicksPerSecond);
            } else {
                launcher.setFlywheelTicks(0);
            }
            launcher.update();
        }

        double currentVelocity = flywheelMotorOne.getVelocity();

        telemetryManager.debug("rawPowerMode", rawPowerMode);
        telemetryManager.debug("shooterEnabled", shooterEnabled);
        telemetryManager.debug("usePIDF", usePIDF);
        telemetryManager.debug("targetTicksPerSecond", targetTicksPerSecond);
        telemetryManager.debug("testPower", testPower);
        telemetryManager.debug("currentVelocity", currentVelocity);
        telemetryManager.debug("P", Launcher23511.P);
        telemetryManager.debug("I", Launcher23511.I);
        telemetryManager.debug("D", Launcher23511.D);
        telemetryManager.debug("F", Launcher23511.F);
        telemetryManager.debug("NOMINAL_VOLTAGE", Launcher23511.NOMINAL_VOLTAGE);
        telemetryManager.update();
    }
}
