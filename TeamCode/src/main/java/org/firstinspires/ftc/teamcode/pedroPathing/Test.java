package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Init;

@TeleOp(name = "Test", group = "0")
public class Test extends OpMode {
    private Follower follower;

    private Init init;
    public static Pose startingPose;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private TelemetryManager telemetryM;


    @Override
    public void init() {
    init.setup();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop(){
        follower.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        telemetry.addData("Heading in Degrees", follower.getHeading());
        telemetry.update();

    }


}
