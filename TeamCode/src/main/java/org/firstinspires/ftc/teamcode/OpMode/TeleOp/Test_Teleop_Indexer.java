package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;

@TeleOp(name = "Test_Teleop_Indexer")
public class Test_Teleop_Indexer extends OpMode {

    private IntakeMotor intkM;
    private Indexer indexer;

    public enum IndexState {
        IndexRight,
        IndexLeft,
        Idle,

        Wait

    }

    ElapsedTime indexTimer = new ElapsedTime();

    IndexState indexState = IndexState.Idle;

    @Override
    public void init() {
        indexer = new Indexer(hardwareMap);
        intkM = new IntakeMotor(hardwareMap);
        indexer.StartIndexPose();
        indexTimer = new ElapsedTime();
        indexTimer.reset();
    }

    @Override
    public void loop() {
            switch (indexState) {
                case IndexLeft:
                    indexer.IndexLeft_PickBall();
                    indexTimer.reset();
                    indexState = IndexState.Wait;

                    break;

                case  IndexRight:
                    indexer.IndexRight_PickBall();
                    break;

                case Wait:
                    if (indexTimer.seconds()>2) {
                        indexState = IndexState.Idle;
                    }
                    break;

                case Idle:
                    indexer.Not_active();
                    if (gamepad1.x){
                        indexState = IndexState.IndexLeft;
                    }
                    if (gamepad1.y){
                        indexState = IndexState.IndexRight;
                    }
                    break;
            }


    }
}
