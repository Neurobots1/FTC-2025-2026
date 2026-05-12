package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants.HardwareMapConstants;

@TeleOp(name = "TEST_HARDWARE_MAP", group = "Tuning")
public class HardwareMapTestTeleOp extends OpMode {

    private static final double DEFAULT_MOTOR_POWER = 0.35;
    private static final double MOTOR_POWER_STEP = 0.05;
    private static final double SERVO_STEP = 0.02;

    private DeviceTest[] devices;
    private int selectedIndex;
    private double selectedMotorPower = DEFAULT_MOTOR_POWER;

    private boolean prevDpadUp;
    private boolean prevDpadDown;
    private boolean prevDpadLeft;
    private boolean prevDpadRight;
    private boolean prevA;
    private boolean prevX;
    private boolean prevY;

    @Override
    public void init() {
        devices = new DeviceTest[]{
                new MotorTest("Drive Left Front", HardwareMapConstants.LEFT_FRONT_DRIVE_MOTOR),
                new MotorTest("Drive Left Rear", HardwareMapConstants.LEFT_REAR_DRIVE_MOTOR),
                new MotorTest("Drive Right Front", HardwareMapConstants.RIGHT_FRONT_DRIVE_MOTOR),
                new MotorTest("Drive Right Rear", HardwareMapConstants.RIGHT_REAR_DRIVE_MOTOR),
                new MotorTest("Intake", HardwareMapConstants.INTAKE_MOTOR),
                new MotorTest("Shooter Left", HardwareMapConstants.SHOOTER_LEFT_FLYWHEEL_MOTOR),
                new MotorTest("Shooter Right", HardwareMapConstants.SHOOTER_RIGHT_FLYWHEEL_MOTOR),
                new MotorTest("Turret", HardwareMapConstants.TURRET_MOTOR),
                new ServoTest("Hood", HardwareMapConstants.HOOD_SERVO),
                new ServoTest("Feed Blocker", HardwareMapConstants.FEED_SERVO),
                new ServoTest("Indexer Left", HardwareMapConstants.INDEX_LEFT_SERVO),
                new ServoTest("Indexer Right", HardwareMapConstants.INDEX_RIGHT_SERVO),
                new ServoTest("Indexer Back Gate", HardwareMapConstants.INDEX_BACK_GATE_SERVO)
        };

        for (DeviceTest device : devices) {
            device.init(hardwareMap);
        }
    }

    @Override
    public void loop() {
        handleSelection();

        DeviceTest selectedDevice = devices[selectedIndex];

        stopAllMotors();

        if (selectedDevice instanceof MotorTest) {
            handleMotorTest((MotorTest) selectedDevice);
        } else if (selectedDevice instanceof ServoTest) {
            handleServoTest((ServoTest) selectedDevice);
        }

        updateEdgeTracking();
        publishTelemetry(selectedDevice);
    }

    @Override
    public void stop() {
        stopAllMotors();
    }

    private void handleSelection() {
        if (gamepad1.dpad_up && !prevDpadUp) {
            selectedIndex = (selectedIndex - 1 + devices.length) % devices.length;
        }

        if (gamepad1.dpad_down && !prevDpadDown) {
            selectedIndex = (selectedIndex + 1) % devices.length;
        }
    }

    private void handleMotorTest(MotorTest motorTest) {
        if (gamepad1.dpad_right && !prevDpadRight) {
            selectedMotorPower = Range.clip(selectedMotorPower + MOTOR_POWER_STEP, 0.10, 1.00);
        }

        if (gamepad1.dpad_left && !prevDpadLeft) {
            selectedMotorPower = Range.clip(selectedMotorPower - MOTOR_POWER_STEP, 0.10, 1.00);
        }

        double power = 0.0;
        if (gamepad1.right_trigger > 0.05) {
            power = selectedMotorPower * gamepad1.right_trigger;
        } else if (gamepad1.left_trigger > 0.05) {
            power = -selectedMotorPower * gamepad1.left_trigger;
        }

        motorTest.setPower(power);
    }

    private void handleServoTest(ServoTest servoTest) {
        if (gamepad1.dpad_right && !prevDpadRight) {
            servoTest.nudge(SERVO_STEP);
        }

        if (gamepad1.dpad_left && !prevDpadLeft) {
            servoTest.nudge(-SERVO_STEP);
        }

        if (gamepad1.a && !prevA) {
            servoTest.setPosition(0.0);
        }

        if (gamepad1.x && !prevX) {
            servoTest.setPosition(0.5);
        }

        if (gamepad1.y && !prevY) {
            servoTest.setPosition(1.0);
        }
    }

    private void publishTelemetry(DeviceTest selectedDevice) {
        telemetry.addData("Selected", "%d / %d", selectedIndex + 1, devices.length);
        telemetry.addData("Label", selectedDevice.label);
        telemetry.addData("Map Name", selectedDevice.mapName);
        telemetry.addData("Type", selectedDevice.typeLabel);
        telemetry.addData("Status", selectedDevice.status);
        telemetry.addData("Value", selectedDevice.getValueSummary());

        if (selectedDevice instanceof MotorTest) {
            telemetry.addData("Motor Power Limit", "%.2f", selectedMotorPower);
            telemetry.addLine("Dpad up/down: choose motor or servo");
            telemetry.addLine("Dpad left/right: lower or raise motor power limit");
            telemetry.addLine("Right trigger: forward, left trigger: reverse");
        } else {
            telemetry.addLine("Dpad up/down: choose motor or servo");
            telemetry.addLine("Dpad left/right: nudge servo by 0.02");
            telemetry.addLine("A = 0.0, X = 0.5, Y = 1.0");
        }

        telemetry.addLine("Only the selected motor is allowed to run.");
        telemetry.update();
    }

    private void stopAllMotors() {
        if (devices == null) {
            return;
        }

        for (DeviceTest device : devices) {
            device.safeStop();
        }
    }

    private void updateEdgeTracking() {
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevA = gamepad1.a;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
    }

    private abstract static class DeviceTest {
        final String label;
        final String mapName;
        final String typeLabel;
        String status = "NOT INIT";

        DeviceTest(String label, String mapName, String typeLabel) {
            this.label = label;
            this.mapName = mapName;
            this.typeLabel = typeLabel;
        }

        abstract void init(HardwareMap hardwareMap);

        abstract String getValueSummary();

        abstract void safeStop();
    }

    private static final class MotorTest extends DeviceTest {
        private DcMotorEx motor;

        MotorTest(String label, String mapName) {
            super(label, mapName, "Motor");
        }

        @Override
        void init(HardwareMap hardwareMap) {
            try {
                motor = hardwareMap.get(DcMotorEx.class, mapName);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setPower(0.0);
                status = "READY";
            } catch (Exception e) {
                motor = null;
                status = "MISSING";
            }
        }

        void setPower(double power) {
            if (motor != null) {
                motor.setPower(power);
            }
        }

        @Override
        String getValueSummary() {
            if (motor == null) {
                return "Device not found in Robot Config";
            }

            return String.format(
                    "power=%.2f encoder=%d current=%.2fA",
                    motor.getPower(),
                    motor.getCurrentPosition(),
                    getCurrentAmps()
            );
        }

        @Override
        void safeStop() {
            setPower(0.0);
        }

        private double getCurrentAmps() {
            if (motor == null) {
                return 0.0;
            }

            try {
                return motor.getCurrent(CurrentUnit.AMPS);
            } catch (Exception ignored) {
                return 0.0;
            }
        }
    }

    private static final class ServoTest extends DeviceTest {
        private Servo servo;
        private double targetPosition = 0.5;

        ServoTest(String label, String mapName) {
            super(label, mapName, "Servo");
        }

        @Override
        void init(HardwareMap hardwareMap) {
            try {
                servo = hardwareMap.get(Servo.class, mapName);
                targetPosition = Range.clip(servo.getPosition(), 0.0, 1.0);
                status = "READY";
            } catch (Exception e) {
                servo = null;
                status = "MISSING";
            }
        }

        void nudge(double delta) {
            setPosition(targetPosition + delta);
        }

        void setPosition(double position) {
            targetPosition = Range.clip(position, 0.0, 1.0);
            if (servo != null) {
                servo.setPosition(targetPosition);
            }
        }

        @Override
        String getValueSummary() {
            if (servo == null) {
                return "Device not found in Robot Config";
            }

            return String.format("target=%.2f actual=%.2f", targetPosition, servo.getPosition());
        }

        @Override
        void safeStop() {
        }
    }
}
