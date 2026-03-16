package org.firstinspires.ftc.teamcode.OpMode.TeleOp;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.TeleOp.CompetitionTeleopRobot;

@Configurable
@TeleOp(name = "RED", group = "Competition")
public class Teleop_Red_13_01_26 extends BaseCompetitionTeleOp {

    private static final double TAG_RESET_COOLDOWN = 5;

    @Override
    protected CompetitionTeleopRobot.Alliance getAlliance() {
        return CompetitionTeleopRobot.Alliance.RED;
    }

    @Override
    protected double getTagResetCooldownSeconds() {
        return TAG_RESET_COOLDOWN;
    }
}

