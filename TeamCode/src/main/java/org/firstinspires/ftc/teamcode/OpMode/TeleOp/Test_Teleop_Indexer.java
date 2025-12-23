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

    private boolean lastX = false;
    private boolean lastY = false;


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

        boolean xPressed = gamepad1.x && !lastX;
        boolean yPressed = gamepad1.y && !lastY;

        lastX = gamepad1.x;
        lastY = gamepad1.y;

        if (xPressed) indexer.StartIndexLeftPick();
        if (yPressed) indexer.StartIndexRightPick();
        if (!indexer.isBusy()) { indexer.Not_active();}

        if (!indexer.isBusy()) {
            if (gamepad1.a) {
                indexer.indexLeftServo.setPosition(Indexer.indexer_L_Engage);
            } else {
                indexer.indexLeftServo.setPosition(Indexer.indexer_L_Retracted);
            }
        }



        indexer.IndexLeft_PickBall();
        indexer.IndexRight_PickBall();

        telemetry.addData("xPressed", xPressed);
        telemetry.addData("yPressed", yPressed);
        telemetry.addData("Left FSM", indexer.getLeftState());
        telemetry.addData("Right FSM", indexer.getRightState());
        telemetry.addData("Ball Timer", indexer.getTimerSeconds());
        telemetry.update();

    }

}

