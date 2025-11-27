package org.firstinspires.ftc.teamcode.OpMode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.ShooterController;
import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector;
import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector.Alliance;

@TeleOp(name = "Shooter_Burst_Test", group = "Test")
public class Shooter_Burst_Test extends OpMode {

    private Follower follower;
    private ShooterController shooter;
    private AllianceSelector.Manager alliance;

    private boolean rbPrev, aPrev, bPrev, yPrev;
    private final ElapsedTime rbTimer = new ElapsedTime();
    private static final double TAP_MAX_MS = 250;

    private boolean spinEnabled = false;

    @Override
    public void init() {

        //follower = Constants.createFollower(hardwareMap);
        shooter  = ShooterController.create(hardwareMap, follower, Alliance.RED);
        shooter.setSpinEnabled(false);
        shooter.setShootHold(false);
        rbTimer.reset();
    }

    @Override
    public void loop() {
        //follower.update();

        boolean rb = gamepad1.right_bumper;
        boolean a  = gamepad1.a;
        boolean b  = gamepad1.b;
        boolean y  = gamepad1.y;

        if (rb && !rbPrev) rbTimer.reset();
        if (!rb && rbPrev && rbTimer.milliseconds() < TAP_MAX_MS) {
            spinEnabled = !spinEnabled;
            shooter.setSpinEnabled(spinEnabled);
        }

        shooter.setShootHold(rb);
        shooter.update();

        if (a && !aPrev) alliance.setAlliance(Alliance.BLUE);
        if (b && !bPrev) alliance.setAlliance(Alliance.RED);
        if (y && !yPrev) alliance.toggle();

        Pose p = follower.getPose();
        telemetry.addData("Pose", "x=%.1f  y=%.1f  h=%.2f", p.getX(), p.getY(), p.getHeading());
        telemetry.addData("Alliance", shooter.getAlliance().name());
        telemetry.addData("Spin", spinEnabled ? "ON" : "OFF");
        telemetry.addLine(shooter.telemetryLine());
        telemetry.update();

        rbPrev = rb;
        aPrev  = a;
        bPrev  = b;
        yPrev  = y;
    }
}
