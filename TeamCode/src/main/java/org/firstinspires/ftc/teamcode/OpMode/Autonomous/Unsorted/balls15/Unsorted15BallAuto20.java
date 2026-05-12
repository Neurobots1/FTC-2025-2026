package org.firstinspires.ftc.teamcode.OpMode.Autonomous.Unsorted.balls15;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpMode.Autonomous.templates.BaseUnsortedAutoTemplate;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.ModularAutoBuilder;

public abstract class Unsorted15BallAuto20 extends BaseUnsortedAutoTemplate {
    public static double GATE_INTAKE_SECONDS = 1;
    public static double SHOOT_POSE_DEPART_DELAY_SECONDS = 0.5;
    public static double GATE_TURN_SETTLE_SECONDS = 0.35;
    public static double LINE_INTAKE_SPEED = 1;

    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;
    private PathChain path7;
    private PathChain path8;
    private PathChain path9;

    @Override
    protected Pose startPose() {
        return paths().pose(new Pose(34.000, 137.000, Math.toRadians(90)));
    }

    @Override
    protected void buildPaths() {
        Pose startPose = new Pose(34.000, 137.000, Math.toRadians(90));
        Pose shotPose = new Pose(59.000, 78.000, Math.toRadians(180));
        Pose line1IntakePose = new Pose(9.463, 70.588);
        Pose line1ControlOut = new Pose(58.227, 27.322);
        Pose line1ControlBack = new Pose(58.432, 27.199);
        Pose line2IntakePose = new Pose(7.000, 35.000);
        Pose line2ControlPose = new Pose(60.000, 30.000);
        Pose line3IntakePose = new Pose(9.000, 8.000);
        Pose line3ControlPose = new Pose(11.000, 47.000);
        Pose finalApproachPose = new Pose(15.000, 85.000);
        Pose finalShotPose = new Pose(46.000, 115.000);

        path1 = line(startPose, shotPose, false);
        path2 = tangentCurve(shotPose, line1ControlOut, line1IntakePose, false);
        path3 = tangentCurve(line1IntakePose, line1ControlBack, shotPose, true);
        path4 = tangentCurve(shotPose, line2ControlPose, line2IntakePose, false);
        path5 = tangentCurve(line2IntakePose, line2ControlPose, shotPose, true);
        path6 = tangentCurve(shotPose, line3ControlPose, line3IntakePose, false);
        path7 = tangentCurve(line3IntakePose, line3ControlPose, shotPose, true);
        path8 = tangentLine(shotPose, finalApproachPose, false);
        path9 = tangentLine(finalApproachPose, finalShotPose, true);
    }

    private PathChain line(Pose blueStart, Pose blueEnd, boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading());

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain tangentLine(Pose blueStart, Pose blueEnd, boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierLine(start, end))
                .setTangentHeadingInterpolation();

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain tangentCurve(Pose blueStart, Pose blueControl, Pose blueEnd, boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose control = paths().pose(blueControl);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setTangentHeadingInterpolation();

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain constantLine(Pose blueStart, Pose blueEnd, double heading, boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierLine(start, end))
                .setConstantHeadingInterpolation(heading);

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    @Override
    protected void buildRoutine() {
        ModularAutoBuilder builder = new ModularAutoBuilder(robot());
        builder
                .enableUnsortedShooter(true)
                .doAction(startIntake())
                .followAsync(path1, 1.0, true)
                .waitForFollowerIdle()
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitSeconds(SHOOT_POSE_DEPART_DELAY_SECONDS)
                .doAction(startIntake())
                .followAsync(path2, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .followAsync(path3, 1.0, true)
                .waitForUnsortedShotZone()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .waitSeconds(SHOOT_POSE_DEPART_DELAY_SECONDS)
                .doAction(startIntake())
                .followAsync(path4, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .followAsync(path5, 1.0, true)
                .waitForUnsortedShotZone()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .waitSeconds(SHOOT_POSE_DEPART_DELAY_SECONDS)
                .doAction(startIntake())
                .followAsync(path6, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .followAsync(path7, 1.0, true)
                .waitForUnsortedShotZone()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .waitSeconds(SHOOT_POSE_DEPART_DELAY_SECONDS)
                .doAction(startIntake())
                .followAsync(path8, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .follow(path9, 1.0, true)
                .doAction(stopIntake())
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .doAction(stopDrive())
                .stopUnsortedShooter();

        scheduler().setAction(builder.build());
    }

    private org.firstinspires.ftc.teamcode.Subsystems.Autonomous.AutoAction startIntake() {
        return Actions.instant(() -> intake().intake());
    }

    private org.firstinspires.ftc.teamcode.Subsystems.Autonomous.AutoAction stopIntake() {
        return Actions.instant(() -> intake().stop());
    }

    private org.firstinspires.ftc.teamcode.Subsystems.Autonomous.AutoAction stopDrive() {
        return Actions.instant(() -> {
            follower().startTeleopDrive();
            follower().setTeleOpDrive(0.0, 0.0, 0.0, true);
        });
    }

    public static class Blue extends Unsorted15BallAuto20 {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.BLUE;
        }
    }

    public static class Red extends Unsorted15BallAuto20 {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.RED;
        }
    }
}
