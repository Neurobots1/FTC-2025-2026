package org.firstinspires.ftc.teamcode.OpMode.TeleOp;



import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TeleopHeadingLock extends OpMode {

    public static Pose startingPose;
    public static Follower follower;
    private TelemetryManager telemetryM;




    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.startTeleopDrive();
}

@Override
public void loop(){

        double Heading= Math.atan2(2,2);

    follower.setTeleOpDrive(
            -gamepad1.left_stick_y ,
            -gamepad1.left_stick_x,
            Heading,
            false // Robot Centric
    );
    }
}
