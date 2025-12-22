package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;

@TeleOp(name = "Test_Teleop_Indexer")
public class Test_Teleop_Indexer extends OpMode {

    private IntakeMotor intkM;
    private Indexer indexer;

    @Override
    public void init() {
        indexer = new Indexer(hardwareMap);
        intkM = new IntakeMotor(hardwareMap);

    }

    @Override
    public void loop() {
        if (gamepad1.x){
            indexer.Caroussel1PickBall();
        }
        else if (gamepad1.y){
            indexer.Caroussel_2_PickBall();
        }
        else indexer.Not_active();

    }
}
