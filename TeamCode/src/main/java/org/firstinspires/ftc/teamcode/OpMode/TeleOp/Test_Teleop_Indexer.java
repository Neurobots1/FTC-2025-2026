package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

@TeleOp(name = "Test_Teleop_Indexer")
public class Test_Teleop_Indexer extends OpMode {

    private IntakeMotor intkM;
    private Indexer indexer;

    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastB = false;   // NEW

    @Override
    public void init() {
        indexer = new Indexer(hardwareMap);
        intkM = new IntakeMotor(hardwareMap);
        indexer.StartIndexPose();
    }

    @Override
    public void loop() {

        boolean xPressed = gamepad1.x && !lastX;
        boolean yPressed = gamepad1.y && !lastY;
        boolean bPressed = gamepad1.b && !lastB;   // NEW

        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastB = gamepad1.b;                        // NEW

        // Existing picks
        if (xPressed) indexer.indexIntake();
        if (yPressed) indexer.OutTake();
        if (bPressed) {indexer.startIndexIntake();}

        // Let FSMs run every loop
        indexer.IndexLeft_PickBall();
        indexer.OutTake();
        indexer.indexIntake();

        // Only do Not_active / manual servo when nothing is running
         /* if (!indexer.isBusy()) {
            indexer.Not_active();

            if (gamepad1.a) {
                indexer.indexLeftServo.setPosition(Indexer.indexer_L_Engage);
            } else {
                indexer.indexLeftServo.setPosition(Indexer.indexer_L_Retracted);
            }
        } */

        telemetry.addData("xPressed", xPressed);
        telemetry.addData("yPressed", yPressed);
        telemetry.addData("bPressed", bPressed);
        telemetry.addData("Left FSM", indexer.getLeftState());
        telemetry.addData("Right FSM", indexer.getRightState());
        telemetry.addData("Intake FSM", indexer.getIndexIntakeState());
        telemetry.addData("Busy", indexer.isBusy());
        telemetry.addData("Timer", indexer.getTimerSeconds());
        telemetry.update();
    }
}


