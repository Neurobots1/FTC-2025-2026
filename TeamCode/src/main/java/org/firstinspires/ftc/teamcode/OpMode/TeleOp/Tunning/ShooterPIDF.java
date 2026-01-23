package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tunning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.FilteredPIDFController;
@Disabled
@Configurable
public class ShooterPIDF extends OpMode {

    private DcMotorEx mA, mB;

    private static final String MOTOR_A_NAME = "ShooterA";
    private static final String MOTOR_B_NAME = "ShooterB";
    private static final DcMotorSimple.Direction MOTOR_A_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction MOTOR_B_DIR = DcMotorSimple.Direction.FORWARD;

    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO    = 1.0;

    private PIDFController pidf;

    public static double targetRpm = 0.0;
    private boolean spin = false;

    private JoinedTelemetry jt;

    @Override
    public void init() {
        jt = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        mA = hardwareMap.get(DcMotorEx.class, MOTOR_A_NAME);
        mB = hardwareMap.get(DcMotorEx.class, MOTOR_B_NAME);

        // External PIDF → don't use RUN_USING_ENCODER
        mA.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        mB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        mA.setDirection(MOTOR_A_DIR);
        mB.setDirection(MOTOR_B_DIR);

        // Coast when off
        mA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidf = new PIDFController(new PIDFCoefficients(
                ShooterTuningConfig.P, ShooterTuningConfig.I, ShooterTuningConfig.D , ShooterTuningConfig.F));
    }

    @Override
    public void loop() {
        double step = ShooterTuningConfig.stepSmall;
        if (gamepad1.right_trigger > 0.5) step = ShooterTuningConfig.stepBig;
        if (gamepad1.dpad_up)    targetRpm += step;
        if (gamepad1.dpad_down)  targetRpm -= step;
        if (gamepad1.left_stick_button) targetRpm = 0;
        targetRpm = clip(targetRpm, 0, 10000);

        if (gamepad1.right_bumper) spin = true;
        if (gamepad1.left_bumper)  spin = false;

        // Live gains from Panels
        pidf.setCoefficients(new PIDFCoefficients( ShooterTuningConfig.P, ShooterTuningConfig.I, ShooterTuningConfig.D,ShooterTuningConfig.F));

        // Measure RPM from ONE motor only (mA)
        double rpm = (mB.getVelocity() / (TICKS_PER_REV * GEAR_RATIO)) * 60.0;

        double out = 0.0;
        if (spin) {
            double err = targetRpm - rpm;
            pidf.updateFeedForwardInput(targetRpm);
            pidf.updateError(err);
            out = clip(pidf.run(), -1, 1);
        }

        // Same magnitude, opposite direction on the second motor
        mA.setPower(out);
        mB.setPower(-out);

        // Telemetry: single RPM only
        jt.addData("Target RPM", "%.0f", targetRpm);
        jt.addData("Actual RPM", "%.0f", rpm);
        jt.addData("Error RPM",  "%.0f", (targetRpm - rpm));
        jt.addData("Spin", spin ? "ON" : "OFF");
        jt.addData("PIDF", "P=%.6f I=%.6f D=%.6f T=%.3f F=%.6f",
                ShooterTuningConfig.P, ShooterTuningConfig.I, ShooterTuningConfig.D, ShooterTuningConfig.F);
        jt.update();
    }

    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
