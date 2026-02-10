package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.OpMode.Autonomous.Auto_Meet_Blue;

@Configurable
@TeleOp
public class Teleop_Blue_13_01_26 extends OpMode {

    private Robot robot;
    private Auto_Meet_Blue autonomeB;
    private JoinedTelemetry jt;
    private TelemetryManager telemetryManager;
    private final ElapsedTime tagResetTimer = new ElapsedTime();
    private static final double TAG_RESET_COOLDOWN = 5;

    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap);
        autonomeB = new Auto_Meet_Blue();
        autonomeB.GetPose();
        robot.setAlliance(Robot.Alliance.BLUE);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        jt = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        tagResetTimer.reset();
    }



    @Override
    public void start() {
        robot.startTeleop();
    }

    @Override
    public void loop() {
        robot.teleopLoop(gamepad1, tagResetTimer, TAG_RESET_COOLDOWN, jt, telemetryManager);
    }
}
