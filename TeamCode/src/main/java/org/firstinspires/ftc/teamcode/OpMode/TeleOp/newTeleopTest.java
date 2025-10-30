package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Shoot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class newTeleopTest extends OpMode {
    public Follower follower;

    public Shoot shooter;


    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        shooter = new Shoot(hardwareMap,
                0.01, 0.0001, 0.001,   // PID for Motor 1
                0.01,0.001,0.001 // PID for Motor 2
        );
    }


    public void loop(){
        if (gamepad1.a){
            shooter.setTargetRPM(2000);
        }

        if (gamepad1.b){
            shooter.setTargetRPM(1000);
        }
        shooter.update();

    }
}
