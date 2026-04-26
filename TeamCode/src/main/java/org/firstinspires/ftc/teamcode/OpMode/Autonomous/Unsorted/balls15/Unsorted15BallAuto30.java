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
    public static double GATE_INTAKE_SECONDS = 1.5;
    public static double GATE_TURN_SETTLE_SECONDS = 0.35;
    public static double LINE_INTAKE_SPEED = 1.0;

    private PathChain toPreloadShot;
    private PathChain toLine2Intake;
    private PathChain line2ToShot;
    private PathChain shotToGateApproach;
    private PathChain gateApproachToCollect;
    private PathChain gateCollectToShot;
    private PathChain shotToGateApproachAgain;
    private PathChain gateApproachToCollectAgain;
    private PathChain gateCollectToFinalShot;
    private PathChain shotToLine1Intake;
    private PathChain line1ToFinalShot;

    @Override
    protected Pose startPose() {
        return paths().pose(new Pose(20.000, 119.000, Math.toRadians(139)));
    }

    @Override
    protected void buildPaths() {
        Pose startPose = new Pose(20.000, 119.000, Math.toRadians(139));
        Pose preloadShotPose = new Pose(46.781, 82.993, Math.toRadians(231));
        Pose cycleShotPose = new Pose(55.000, 77.000, Math.toRadians(231));
        Pose line2IntakeControl = new Pose(44.222, 49.977);
        Pose line2IntakePose = new Pose(8.000, 52.000);
        Pose line2ReturnControl = new Pose(44.075, 50.183);
        Pose gateApproachControl = new Pose(41.000, 56.000);
        Pose gateApproachPose = new Pose(11.750, 56.000, Math.toRadians(180));
        Pose gateCollectPose = new Pose(10.000, 47.000, Math.toRadians(140));
        Pose line1IntakePose = new Pose(14.000, 83.000);
        Pose finalShotPose = new Pose(60.207, 100.689);

        toPreloadShot = line(startPose, preloadShotPose, false);
        toLine2Intake = tangentCurve(preloadShotPose, line2IntakeControl, line2IntakePose, false);
        line2ToShot = tangentCurve(line2IntakePose, line2ReturnControl, cycleShotPose, true);
        shotToGateApproach = tangentCurve(cycleShotPose, gateApproachControl, gateApproachPose, false);
        gateApproachToCollect = line(gateApproachPose, gateCollectPose, false);
        gateCollectToShot = tangentLine(gateCollectPose, cycleShotPose, true);
        shotToGateApproachAgain = tangentCurve(cycleShotPose, gateApproachControl, gateApproachPose, false);
        gateApproachToCollectAgain = line(gateApproachPose, gateCollectPose, false);
        gateCollectToFinalShot = tangentLine(gateCollectPose, cycleShotPose, true);
        shotToLine1Intake = tangentLine(cycleShotPose, line1IntakePose, false);
        line1ToFinalShot = tangentLine(line1IntakePose, finalShotPose, true);
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

    private PathChain curve(Pose blueStart, Pose blueControl, Pose blueEnd, boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose control = paths().pose(blueControl);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading());

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
                .followAsync(toPreloadShot, 1.0, true)
                .doAction(startTimedIntake())
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .followAsync(toLine2Intake, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .followAsync(line2ToShot, 1.0, true)
                .doAction(stopIntake())
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .followAsync(shotToGateApproach, 1.0, true)
                .waitForFollowerIdle()
                .followAsync(gateApproachToCollect, 1.0, false)
                .waitForFollowerIdle()
                .doAction(Actions.waitSeconds(GATE_TURN_SETTLE_SECONDS))
                .doAction(Actions.waitSeconds(GATE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .followAsync(gateCollectToShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .followAsync(shotToGateApproachAgain, 1.0, true)
                .waitForFollowerIdle()
                .followAsync(gateApproachToCollectAgain, 1.0, false)
                .waitForFollowerIdle()
                .doAction(Actions.waitSeconds(GATE_TURN_SETTLE_SECONDS))
                .doAction(Actions.waitSeconds(GATE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .followAsync(gateCollectToFinalShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .followAsync(shotToLine1Intake, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .doAction(stopIntake())
                .followAsync(line1ToFinalShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .stopUnsortedShooter();

        scheduler().setAction(builder.build());
    }

    private org.firstinspires.ftc.teamcode.Subsystems.Autonomous.AutoAction startTimedIntake() {
        return Actions.instant(() -> intake().intake());
    }

    private org.firstinspires.ftc.teamcode.Subsystems.Autonomous.AutoAction stopIntake() {
        return Actions.instant(() -> intake().stop());
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
