package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;

//@TeleOp(name = "Index_Test")
public class Index_Test extends OpMode {

    private Indexer_Base indexerBase;

    // previous states
    private boolean lastRight = false;
    private boolean lastLeft  = false;
    private boolean lastUp    = false;
    private boolean lastDown  = false;

    @Override
    public void init() {
        indexerBase = new Indexer_Base(hardwareMap);
        indexerBase.StartIndexPose();
    }

    @Override
    public void loop() {

        // current states
        boolean right = gamepad1.dpad_right;
        boolean left  = gamepad1.dpad_left;
        boolean up    = gamepad1.dpad_up;
        boolean down  = gamepad1.dpad_down;

        // rising edges
        if (right && !lastRight) {
            indexerBase.indexRightServo.setPosition(1);
        }

        if (up && !lastUp) {
            indexerBase.indexRightServo.setPosition(0);
        }

        if (down && !lastDown) {
            indexerBase.indexLeftServo.setPosition(1);
        }

        if (left && !lastLeft) {
            indexerBase.indexLeftServo.setPosition(0);
        }

        // update last states
        lastRight = right;
        lastLeft  = left;
        lastUp    = up;
        lastDown  = down;
    }
}