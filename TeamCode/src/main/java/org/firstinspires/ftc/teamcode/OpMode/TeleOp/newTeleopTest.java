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

    public static double p = 0.1;

    public static double i;

    public static double d;

    private TelemetryManager telemetryM;
    private PanelsTelemetry panelsTelemetry;

    public void init(){

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    public void loop(){

        shooter = new Shoot( hardwareMap,
                p, i, d
        );
        if (gamepad1.a){
            Target= Target+100;
        }

        if (gamepad1.b){
            Target = Target-100;
        }

        shooter.setTargetRPM(Target);
        shooter.update();
        telemetryM.debug("TargetRpm", shooter.getTargetRPM());
        telemetryM.debug("averageRpm", shooter.getAverageRPM());
        telemetryM.addData("CurrentRpm1", shooter.getCurrentRPM1());;
        telemetryM.update();
    }
}
