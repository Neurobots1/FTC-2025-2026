package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PGP;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Test_Teleop_Index_PGP", group = "Test")
public class Test_Teleop_Index extends OpMode {

    private Indexer_Base base;
    private LauncherSubsystem shooter;
    private Indexer_PGP pgp;

    private Follower follower;

    private final Pose startingPose = new Pose(72, 72, Math.toRadians(90));

    private final Gamepad last = new Gamepad();

    private boolean pressed(boolean now, boolean prev) {
        return now && !prev;
    }

    private double getDistanceToGoal() {
        double gx = 12;
        double gy = 132;
        Pose p = follower.getPose();
        double dx = gx - p.getX();
        double dy = gy - p.getY();
        return Math.hypot(dx, dy);
    }

    @Override
    public void init() {
        base = new Indexer_Base(hardwareMap);
        base.StartIndexPose();

        shooter = LauncherSubsystem.create(hardwareMap);
        shooter.setBlocker(hardwareMap.get(Servo.class, "Blocker"));
        shooter.init();

        pgp = new Indexer_PGP(hardwareMap, base, shooter);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);


        telemetry.addLine("=== INDEXER PGP BUTTON TEST ===");
        telemetry.addLine("");
        telemetry.addLine("Button Mapping:");
        telemetry.addLine(" A     -> Line 1 Intake");
        telemetry.addLine(" B     -> Line 1 Outtake");
        telemetry.addLine(" X     -> Line 2 Intake");
        telemetry.addLine(" Y     -> Line 2 Outtake");
        telemetry.addLine(" LB    -> Line 3 Intake");
        telemetry.addLine(" RB    -> Line 3 Outtake");
        telemetry.addLine(" BACK  -> STOP ALL (reset states + intake off + flywheel off)");
        telemetry.addLine("");
        telemetry.addLine("Pose start = (72, 72, 90deg)");
        telemetry.addLine("Press once to start a sequence.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.getPose();
        double distance = getDistanceToGoal();

        boolean a    = pressed(gamepad1.a, last.a);
        boolean b    = pressed(gamepad1.b, last.b);
        boolean x    = pressed(gamepad1.x, last.x);
        boolean y    = pressed(gamepad1.y, last.y);
        boolean lb   = pressed(gamepad1.left_bumper, last.left_bumper);
        boolean rb   = pressed(gamepad1.right_bumper, last.right_bumper);
        boolean back = pressed(gamepad1.back, last.back);

        if (a)  pgp.startLine1Intake();
        if (b)  pgp.startLine1Outtake();
        if (x)  pgp.startLine2Intake();
        if (y)  pgp.startLine2Outtake();
        if (lb) pgp.startLine3Intake();
        if (rb) pgp.startLine3Outtake();

        if (back) {
            pgp.stopAll();
        }

        pgp.setShootContext(pose.getX(), pose.getY(), distance);
        pgp.update();

        telemetry.addLine("=== BUTTONS ===");
        telemetry.addData("A",    "Line1 Intake");
        telemetry.addData("B",    "Line1 Outtake (uses shooter auto state machine)");
        telemetry.addData("X",    "Line2 Intake");
        telemetry.addData("Y",    "Line2 Outtake (uses shooter auto state machine)");
        telemetry.addData("LB",   "Line3 Intake");
        telemetry.addData("RB",   "Line3 Outtake (uses shooter auto state machine)");
        telemetry.addData("BACK", "STOP ALL");

        telemetry.addLine("");
        telemetry.addLine("=== POSE ===");
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("headingDeg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("distToGoal", distance);

        telemetry.addLine("");
        telemetry.addLine("=== INDEXER STATES ===");
        telemetry.addData("Busy",  pgp.isBusy());
        telemetry.addData("L1",    pgp.pgpState1);
        telemetry.addData("L1_OT", pgp.pgpState1_OT);
        telemetry.addData("L2",    pgp.pgpState2);
        telemetry.addData("L2_OT", pgp.pgpState2_OT);
        telemetry.addData("L3",    pgp.pgpState3);
        telemetry.addData("L3_OT", pgp.pgpState3_OT);

        telemetry.addLine("");
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Target TPS", shooter.getTargetTPS());
        telemetry.addData("Ready", shooter.flywheelReady());

        telemetry.update();

        last.copy(gamepad1);
    }
}

