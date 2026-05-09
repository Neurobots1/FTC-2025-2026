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

public abstract class Unsorted15BallAuto30 extends BaseUnsortedAutoTemplate {
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
    private PathChain path100;
    private PathChain path9;
    private PathChain path10;
    private PathChain path11;
    private PathChain path12;
    private PathChain path13;
    private PathChain path14;
    private PathChain path15;

    @Override
    protected Pose startPose() {
        return paths().pose(new Pose(34.000, 135.000, Math.toRadians(270)));
    }

    @Override
    protected void buildPaths() {
        Pose startPose = new Pose(34.609, 137.659, Math.toRadians(270));
        Pose firstShotPose = new Pose(54.260, 93.945, Math.toRadians(270));
        Pose line1IntakePose = new Pose(17.993, 85.071, Math.toRadians(175));
        Pose shotPose = new Pose(59.19, 78.04);
        Pose line2ControlPose = new Pose(60.468, 59.166);
        Pose line2IntakePose = new Pose(7.305, 52.900);
        Pose line3ControlPose = new Pose(38.000, 60.034);
        Pose gatePose = new Pose(11, 51.000, Math.toRadians(140));
        Pose gatePose2 = new Pose(9, 54.000, Math.toRadians(180));
        Pose finalPose = new Pose(55.000, 108.000);
        Pose line3IntakePose = new Pose(18, 64.034, Math.toRadians(180));

        path1 = line(startPose, firstShotPose, false);
        path2 = tangentLine(firstShotPose, line1IntakePose, false);
        path3 = tangentLine(line1IntakePose, shotPose, true);
        path4 = tangentCurve(shotPose, line2ControlPose, line2IntakePose, false);
        path5 = tangentLine(line2IntakePose, shotPose, true);
        path6 = constantLine(shotPose, line3IntakePose,Math.toRadians(180),false);
        path7 = line(line3IntakePose, gatePose, false);
        path8 = line(gatePose, gatePose2, false);
        path9 = tangentLine(gatePose2, shotPose, true);
        path10 = constantLine(shotPose, line3IntakePose, Math.toRadians(180), false);
        path11 = line(line3IntakePose, gatePose, false);
        path12 = line(gatePose, gatePose2, false);
        path13 = tangentLine(gatePose2, finalPose, true);
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
                .followAsync(path7, 1.0, false)
                .waitForFollowerIdle()
                .doAction(Actions.waitSeconds(GATE_TURN_SETTLE_SECONDS))
                .doAction(Actions.waitSeconds(GATE_INTAKE_SECONDS))
                .followAsync(path8, 1.0, false)
                .waitSeconds(0.3)
                .doAction(stopIntake())
                .followAsync(path9, LINE_INTAKE_SPEED, false)
                .waitForUnsortedShotZone()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .waitSeconds(SHOOT_POSE_DEPART_DELAY_SECONDS)
                .doAction(startIntake())
                .followAsync(path10, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(path11, 1.0, true)
                .waitForFollowerIdle()
                .doAction(Actions.waitSeconds(GATE_TURN_SETTLE_SECONDS))
                .doAction(Actions.waitSeconds(GATE_INTAKE_SECONDS))
                .followAsync(path12, 1.0, false)
                .waitSeconds(0.3)
                .doAction(stopIntake())
                .follow(path13, 1.0, true)
                .doAction(stopDrive())
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

    public static class Blue extends Unsorted15BallAuto30 {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.BLUE;
        }
    }

    public static class Red extends Unsorted15BallAuto30 {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.RED;
        }
    }
}
