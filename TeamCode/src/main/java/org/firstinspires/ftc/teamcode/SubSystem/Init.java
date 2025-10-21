package org.firstinspires.ftc.teamcode.SubSystem;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Init {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private TelemetryManager telemetryM;

    private final Pose startPose = new Pose(63, 136, Math.toRadians(90)); // Start Pose of our robot.



/* public void setup(){
tuning = new Tuning();
    pathTimer = new Timer();
    actionTimer = new Timer();
    opmodeTimer = new Timer();
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    follower = Constants.createFollower(hardwareMap);
    follower.update();
} */

}
