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

    public static double p1 = 0.1;

    public static double i1;

    public static double d1;

    public static double p2;

    public static double i2;

    public static double d2 ;



    private TelemetryManager telemetryM;


    public void init(){

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    public void loop(){

        shooter = new Shoot( hardwareMap,
                p1, i1, d1,
                p2, i2, d2
        );
        if (gamepad1.a){
            Target= Target+100;
        }

        if (gamepad1.b){
            Target = Target-100;
        }

        shooter.setTargetRPM(Target);
        telemetryM.addData("TargetRpm", shooter.getTargetRPM());
        telemetryM.addData("averageRpm", shooter.getAverageRPM());
        telemetryM.debug("TargetRpm", shooter.getTargetRPM());
        telemetryM.debug("averageRpm", shooter.getAverageRPM());
        telemetryM.addData("p1", p1);
        telemetryM.update();
        shooter.update();



    }
}
