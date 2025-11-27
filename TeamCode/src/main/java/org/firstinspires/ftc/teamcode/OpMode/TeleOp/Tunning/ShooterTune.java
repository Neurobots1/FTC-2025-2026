package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Tunning;




import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.ShooterController;

@TeleOp
@Configurable
public class ShooterTune  extends OpMode {



    private ShooterController shooter;
    private TelemetryManager telemetryM;
    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // ShooterController handles ALL hardware mapping internally
        shooter = ShooterController.create(
                hardwareMap, null, AllianceSelector.Alliance.RED
        );

        // Always enabled for tuning
        shooter.setSpinEnabled(true);
        shooter.setShootHold(false);
    }

    @Override
    public void loop() {
        // Use the tuning rpm directly from ShooterController (Panels-editable)
        shooter.tuneRPM(ShooterController.TARGET_RPM);

        shooter.addTuningTelemetry(telemetry);
        telemetryM.addData("shooter Rpm", shooter.getShooterRpm());
        telemetryM.addData("Target Rpm", ShooterController.TARGET_RPM);
        telemetry.update();
        telemetryM.update();

    }

    @Override
    public void stop() {
        shooter.tuneRPM(0);
        shooter.setSpinEnabled(false);
    }
    }


