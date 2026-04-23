package org.firstinspires.ftc.teamcode.OpMode.Autonomous.Unsorted.balls15;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Constants.AutoPoseConstants;
import org.firstinspires.ftc.teamcode.OpMode.Autonomous.templates.BaseUnsortedAutoTemplate;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.ModularAutoBuilder;

public abstract class Unsorted15BallAuto20 extends BaseUnsortedAutoTemplate {
    public static double GATE_INTAKE_SECONDS = 1.5;
    public static double GATE_TURN_SETTLE_SECONDS = 0.35;
    public static double LINE_INTAKE_SPEED = 1.0;

    private PathChain toPreloadShot;
    private PathChain toLine2Intake;
    private PathChain line2ToShot;
    private PathChain shotToGate;
    private PathChain gateToShot;
    private PathChain shotToGateAgain;
    private PathChain gateToFinalShot;
    private PathChain shotToLine1Intake;
    private PathChain line1ToFinalShot;

    @Override
    protected Pose startPose() {
        return paths().pose(new Pose(20.000, 119.000, Math.toRadians(139)));
    }

    @Override
    protected void buildPaths() {
        Pose startPose = new Pose(20.000, 119.000, Math.toRadians(139));
        Pose preloadShotPose = new Pose(62.000, 76.000, Math.toRadians(231));
        Pose cycleShotPose = new Pose(58.000, 76.000, Math.toRadians(231));
        Pose line2IntakeControl = new Pose(43.811, 50.800);
        Pose line2IntakePose = new Pose(8.000, 50.664);
        Pose line2ReturnControl = new Pose(40.500, 50.668);
        Pose gateApproachControl = new Pose(40.000, 30.000);
        Pose gateReturnControl = new Pose(40.000, 30.000);
        Pose gatePose = new Pose(12.000, 47.000, Math.toRadians(143));
        Pose line1IntakeControl = new Pose(40.344, 85.189);
        Pose line1IntakePose = new Pose(14.000, 86.000);
        Pose finalShotPose = new Pose(60.000, 98.000);

        toPreloadShot = line(startPose, preloadShotPose, false);
        toLine2Intake = tangentCurve(preloadShotPose, line2IntakeControl, line2IntakePose, false);
        line2ToShot = tangentCurve(line2IntakePose, line2ReturnControl, cycleShotPose, true);
        shotToGate = curve(cycleShotPose, gateApproachControl, gatePose, false);
        gateToShot = curve(gatePose, gateReturnControl, cycleShotPose, false);
        shotToGateAgain = curve(cycleShotPose, gateApproachControl, gatePose, false);
        gateToFinalShot = curve(gatePose, gateReturnControl, cycleShotPose, false);
        shotToLine1Intake = tangentCurve(cycleShotPose, line1IntakeControl, line1IntakePose, false);
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
                .followAsync(shotToGate, 1.0, true)
                .waitForFollowerIdle()
                .doAction(Actions.waitSeconds(GATE_TURN_SETTLE_SECONDS))
                .doAction(Actions.waitSeconds(GATE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .followAsync(gateToShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .followAsync(shotToGateAgain, 1.0, true)
                .waitForFollowerIdle()
                .doAction(Actions.waitSeconds(GATE_TURN_SETTLE_SECONDS))
                .doAction(Actions.waitSeconds(GATE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .followAsync(gateToFinalShot, 1.0, true)
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
