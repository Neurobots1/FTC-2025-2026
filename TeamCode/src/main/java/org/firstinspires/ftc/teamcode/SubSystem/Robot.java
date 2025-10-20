package org.firstinspires.ftc.teamcode.SubSystem;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class Robot {
    public Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;

    public TelemetryManager telemetryM;

    public DcMotorEx shootMotor1;
    public DcMotorEx shootMotor2;

    public DcMotorEx climbMotor;

    public HardwareMap hardwareMap;


    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //FuturClimbMotor
        /*climbMotor = hardwareMap.get(DcMotorEx.class, "climbMotor");
        climbMotor.setDirection(DcMotorSimple.Direction.FORWARD); */


    }



}

