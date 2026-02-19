package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Test_Teleop_Index_Rapid", group = "Test")
@SuppressWarnings("all")
public class Test_Teleop_Index_Rapid extends OpMode {

    private Indexer_Base base;
    private LauncherSubsystem shooter;
    private Indexer_Rapid rapid;

    private Follower follower;

    private static final Pose STARTING_POSE = new Pose(72, 72, Math.toRadians(90));
    private static final double GOAL_X = 12;
    private static final double GOAL_Y = 132;

    private final Gamepad last = new Gamepad();

    private boolean pressed(boolean now, boolean prev) {
        return now && !prev;
    }

    private double distanceToGoal(Pose p) {
        double dx = GOAL_X - p.getX();
        double dy = GOAL_Y - p.getY();
        return Math.hypot(dx, dy);
    }

    @Override
    public void init() {
        base = new Indexer_Base(hardwareMap);
        base.StartIndexPose();

        shooter = LauncherSubsystem.create(hardwareMap);
        shooter.setBlocker(hardwareMap.get(Servo.class, "Blocker"));
        shooter.init();

        rapid = new Indexer_Rapid(hardwareMap, base, shooter);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(STARTING_POSE);

        telemetry.addLine("=== INDEXER RAPID BUTTON TEST ===");
        telemetry.addLine("");
        telemetry.addLine("Button Mapping:");
        telemetry.addLine(" A     -> Rapid Intake (No Sort)");
        telemetry.addLine(" B     -> Rapid Outtake (Shoot)");
        telemetry.addLine(" X     -> Setup Outtake (wait for flywheel ready)");
        telemetry.addLine(" Y     -> Finish Outtake (feed for ~1.7s when ready)");
        telemetry.addLine(" BACK  -> STOP ALL (reset states + intake off + flywheel off)");
        telemetry.addLine("");
        telemetry.addLine("Pose start = (72, 72, 90deg)");
        telemetry.addLine("Press once to start a sequence.");
        telemetry.update();

        last.copy(gamepad1);
    }

    @Override
    public void loop() {
        if (follower != null) follower.update();

        Pose pose = (follower != null) ? follower.getPose() : STARTING_POSE;
        double distance = distanceToGoal(pose);

        boolean a    = pressed(gamepad1.a, last.a);
        boolean b    = pressed(gamepad1.b, last.b);
        boolean x    = pressed(gamepad1.x, last.x);
        boolean y    = pressed(gamepad1.y, last.y);
        boolean back = pressed(gamepad1.back, last.back);

        if (a) rapid.startRapidIntakeNoSort();
        if (b) rapid.startRapidOuttake();
        if (x) rapid.startSetupOuttake();
        if (y) rapid.startFinishOuttake();

        if (back) rapid.stopAll();

        rapid.setShootContext(pose.getX(), pose.getY(), distance);
        rapid.update();

        telemetry.addLine("=== BUTTONS ===");
        telemetry.addData("A",    "Rapid Intake (No Sort)");
        telemetry.addData("B",    "Rapid Outtake (Shoot)");
        telemetry.addData("X",    "Setup Outtake");
        telemetry.addData("Y",    "Finish Outtake");
        telemetry.addData("BACK", "STOP ALL");

        telemetry.addLine("");
        telemetry.addLine("=== POSE ===");
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("headingDeg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("distToGoal", distance);

        telemetry.addLine("");
        telemetry.addLine("=== INDEXER ===");
        telemetry.addData("Busy", rapid.isBusy());

        telemetry.addLine("");
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Target TPS", shooter.getTargetTPS());
        telemetry.addData("Ready", shooter.flywheelReady());

        telemetry.update();

        last.copy(gamepad1);
    }

    @Override
    public void stop() {
        if (rapid != null) rapid.stopAll();
    }
}

