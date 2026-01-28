package org.firstinspires.ftc.teamcode.SubSystem.Shooter;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

import org.firstinspires.ftc.teamcode.SubSystem.Shooter.ShooterTuner;

@Configurable
@TeleOp(name = "TeleOp ShooterTuner")
public class TeleOp_ShooterTuner extends OpMode {

    private ShooterTuner shooter;
    private IntakeMotor intake;

    // If you want to switch alliance from dashboard
    public static ShooterTuner.Alliance ALLIANCE = ShooterTuner.Alliance.BLUE;

    // Field-centric (optional)
    public static boolean FIELD_CENTRIC = false;
    public static double FIELD_CENTRIC_OFFSET = 0.0;

    @Override
    public void init() {
        shooter = new ShooterTuner();
        shooter.init(hardwareMap);
        shooter.setAlliance(ALLIANCE);
        intake = new IntakeMotor(hardwareMap);
    }

    @Override
    public void start() {
        shooter.setAlliance(ALLIANCE);
        shooter.startTeleop();
    }

    @Override
    public void loop() {
        // Drive (Pedro)
        double driveY = -gamepad1.left_stick_y;
        double driveX = -gamepad1.left_stick_x;
        double turn   = -gamepad1.right_stick_x;

        if (gamepad1.right_bumper) intake.intake();
        else if (gamepad1.left_bumper) intake.outtake();
        else intake.stop();
        // Shooter controls
        boolean shootToggle = gamepad1.y;
        boolean tpsUp = gamepad1.dpad_up;
        boolean tpsDown = gamepad1.dpad_down;

        shooter.updateLoop(
                driveY, driveX, turn,
                FIELD_CENTRIC, FIELD_CENTRIC_OFFSET,
                shootToggle,
                tpsUp, tpsDown
        );

        telemetry.addData("Alliance", ALLIANCE);
        telemetry.addData("Pose", shooter.getPose());
        telemetry.addData("DistanceToGoal", "%.2f", shooter.getDistanceToGoal());

        telemetry.addData("ShootState", shooter.getShootState());
        telemetry.addData("ManualTPS", "%.0f", ShooterTuner.MANUAL_TPS);
        telemetry.addData("TargetTPS", "%.0f", shooter.getTargetTPS());
        telemetry.addData("CurrentTPS", "%.0f", shooter.getCurrentTPS());
        telemetry.addData("AtSpeed", shooter.isAtSpeed());

        telemetry.update();
    }
}

