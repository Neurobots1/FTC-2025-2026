package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
@Autonomous(name = "Auto_Index", group = "Examples")
public class Auto_Index extends OpMode {
    public Indexer indexer;
    public IntakeMotor intkM;


    public void init(){
        indexer = new Indexer(hardwareMap);

    }
    public void loop(){
        indexer.GPP();
    }
}
