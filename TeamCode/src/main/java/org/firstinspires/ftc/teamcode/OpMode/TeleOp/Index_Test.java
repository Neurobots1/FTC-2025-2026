package org.firstinspires.ftc.teamcode.OpMode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;

@TeleOp(name = "Index_Test")
public class Index_Test extends OpMode {

    private Indexer indexer;
    @Override
    public void init() {
        indexer = new Indexer(hardwareMap);
        indexer.StartIndexPose();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()){
            indexer.indexRightServo.setPosition(1);
        }
        if (gamepad1.dpadLeftWasPressed()){
            indexer.indexLeftServo.setPosition(0);
        }
        if (gamepad1.dpadDownWasPressed()){
            indexer.indexLeftServo.setPosition(1);
        }
        if (gamepad1.dpadUpWasPressed()){
            indexer.indexRightServo.setPosition(0);
        }


    }
}
