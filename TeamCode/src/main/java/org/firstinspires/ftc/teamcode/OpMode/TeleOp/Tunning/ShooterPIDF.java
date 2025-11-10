package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tunning;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.FilteredPIDFController;


import org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tunning.ShooterTuningConfig;

@TeleOp (name = "Shooter_PIDF_Minimal", group = "Tuning")
public class ShooterPIDF extends OpMode {





        private DcMotorEx mA, mB;

        private static final String MOTOR_A_NAME = "shootMotor1";
        private static final String MOTOR_B_NAME = "shootMotor2";
        private static final DcMotorSimple.Direction MOTOR_A_DIR = DcMotorSimple.Direction.FORWARD;
        private static final DcMotorSimple.Direction MOTOR_B_DIR = DcMotorSimple.Direction.FORWARD;

        private static final double TICKS_PER_REV = 28.0;
        private static final double GEAR_RATIO    = 1.0;

        private FilteredPIDFController pidf;

        private double targetRpm = 0.0;
        private boolean spin = false;

    private JoinedTelemetry joinedTelemetry;

    joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry);


    @Override
        public void init() {
            mA = hardwareMap.get(DcMotorEx.class, MOTOR_A_NAME);
            mB = hardwareMap.get(DcMotorEx.class, MOTOR_B_NAME);
            mA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mA.setDirection(MOTOR_A_DIR);
            mB.setDirection(MOTOR_B_DIR);

            pidf = new FilteredPIDFController(new FilteredPIDFCoefficients(
                    ShooterTuningConfig.P, ShooterTuningConfig.I, ShooterTuningConfig.D,
                    ShooterTuningConfig.T, ShooterTuningConfig.F));
        }

        @Override
        public void loop() {
            // Target RPM from controller
            double step = ShooterTuningConfig.stepSmall;
            if (gamepad1.right_trigger > 0.5) step = ShooterTuningConfig.stepBig;
            if (gamepad1.dpad_up)    targetRpm += step;
            if (gamepad1.dpad_down)  targetRpm -= step;
            if (gamepad1.left_stick_button) targetRpm = 0;
            targetRpm = clip(targetRpm, 0, 10000);

            // Spin toggle
            if (gamepad1.right_bumper) spin = true;
            if (gamepad1.left_bumper)  spin = false;

            // Apply latest gains from Panels every loop
            pidf.setCoefficients(new FilteredPIDFCoefficients(
                    ShooterTuningConfig.P, ShooterTuningConfig.I, ShooterTuningConfig.D,
                    ShooterTuningConfig.T, ShooterTuningConfig.F));

            double rpm = getAvgRpm();
            double out = 0.0;
            if (spin) {
                double err = targetRpm - rpm;
                pidf.updateFeedForwardInput(targetRpm);
                pidf.updateError(err);
                out = clip(pidf.run(), -1, 1);
            }

            // Opposed power since your two motors are belted together
            mA.setPower(out);
            mB.setPower(-out);

            // Mirror to Panels + DriverHub
            jt.addData("Target RPM", "%.0f", targetRpm);
            jt.addData("Actual RPM", "%.0f", rpm);
            jt.addData("Error RPM",  "%.0f", (targetRpm - rpm));
            jt.addData("Spin", spin ? "ON" : "OFF");
            jt.addData("PIDF", "P=%.6f I=%.6f D=%.6f T=%.3f F=%.6f",
                    ShooterTuningConfig.P, ShooterTuningConfig.I, ShooterTuningConfig.D,
                    ShooterTuningConfig.T, ShooterTuningConfig.F);
            jt.update();
        }

        private double getAvgRpm() {
            double rpsA = mA.getVelocity() / (TICKS_PER_REV * GEAR_RATIO);
            double rpsB = mB.getVelocity() / (TICKS_PER_REV * GEAR_RATIO);
            return ((rpsA + rpsB) * 0.5) * 60.0;
        }

        private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    }


