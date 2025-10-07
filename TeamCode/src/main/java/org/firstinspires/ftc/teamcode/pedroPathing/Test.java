package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.reflect.Field;

@TeleOp(name = "Test", group = "0")
public class Test extends OpMode {
    private Follower follower;
    public static Pose startingPose;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);

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
