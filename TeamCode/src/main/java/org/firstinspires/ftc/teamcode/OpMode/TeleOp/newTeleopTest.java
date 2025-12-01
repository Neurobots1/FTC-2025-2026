package org.firstinspires.ftc.teamcode.OpMode.TeleOp;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Shoot;

@Configurable
@TeleOp
public class newTeleopTest extends OpMode {


    public Shoot shooter;
    public static double Target = 0;

    private TelemetryManager telemetryM;
    private PanelsTelemetry panelsTelemetry;

    public void init(){

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shoot( hardwareMap);

    }


    public void loop(){

        shooter.setTargetRPM(Target);
        telemetryM.addData("shooter Rpm", shooter.getAverageRPM());
        telemetryM.addData("Target Rpm", shooter.getTargetRPM());
        telemetry.update();
        telemetryM.update();
        shooter.update();
    }
}
