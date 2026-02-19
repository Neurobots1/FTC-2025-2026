package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.SubSystem.Robot2;


@Configurable
@TeleOp
public class Test_Auto_PoseSaved extends OpMode {

    private Robot2 robot;
    private JoinedTelemetry jt;
    public Follower follower;
    private TelemetryManager telemetryManager;
    private final ElapsedTime tagResetTimer = new ElapsedTime();
    private static final double TAG_RESET_COOLDOWN = 5;

    @Override
    public void init() {
        robot = new Robot2();
        robot.init(hardwareMap);

        robot.setAlliance(Robot2.Alliance.BLUE);

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
