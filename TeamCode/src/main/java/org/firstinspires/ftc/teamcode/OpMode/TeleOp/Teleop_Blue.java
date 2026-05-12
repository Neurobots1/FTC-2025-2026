package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.TeleOp.CompetitionTeleopRobot;

@Configurable
@TeleOp(name = "BLUE", group = "Competition")
public class Teleop_Blue extends BaseCompetitionTeleOp {

    private static final double TAG_RESET_COOLDOWN = 0.5;

    @Override
    protected CompetitionTeleopRobot.Alliance getAlliance() {
        return CompetitionTeleopRobot.Alliance.BLUE;
    }

    @Override
    protected double getTagResetCooldownSeconds() {
        return TAG_RESET_COOLDOWN;
    }
}
