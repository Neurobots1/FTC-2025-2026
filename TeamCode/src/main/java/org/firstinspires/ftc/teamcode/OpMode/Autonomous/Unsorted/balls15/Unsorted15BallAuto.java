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

public abstract class Unsorted15BallAuto extends BaseUnsortedAutoTemplate {
    public static double GATE_INTAKE_SECONDS = 2;
    public static double LINE_INTAKE_SPEED = 1;

    private PathChain toPreloadShot;
    private PathChain toLine2Control;
    private PathChain line2ControlToStart;
    private PathChain toLine2Start;
    private PathChain toLine2Finish;
    private PathChain line2ToShot;
    private PathChain shotToGate;
    private PathChain gateToShot;
    private PathChain shotToGateAgain;
    private PathChain openGate;
    private PathChain gateToFinalShot;
    private PathChain shotToLine1Start;
    private PathChain line1StartToFinish;
    private PathChain intakeRightLine;
    private PathChain line1ToFinalShot;

    @Override
    protected Pose startPose() {
        return paths().pose(new Pose(56.000, 8.500, Math.toRadians(180)));
    }

    @Override
    protected void buildPaths() {
        Pose startPose = new Pose(56.000, 8.500, Math.toRadians(180));
        Pose humanZone = new Pose(12.000, 8.500);
        Pose humanZoneShot = new Pose(48.000, 13.000);
        Pose leftLine = new Pose(18.444, 35.183);
        Pose leftLineShot = new Pose(47.989, 13.164);
        Pose middleLine = new Pose(23.597, 63.974);
        Pose openGatePose = new Pose(15.728, 63.939);
        Pose middleLineShot = new Pose(50.233, 78.124);
        Pose gatePose = new Pose(9.058, 58.000);
        Pose gateShot = new Pose(50.041, 83.121);
        Pose rightLine = new Pose(21.932, 82.739);
        Pose finalShot = new Pose(44.475, 99.469);

        toPreloadShot = linearHeadingLine(
                startPose,
                startPose,
                Math.toRadians(180),
                Math.toRadians(180),
                false
        );
        line2ControlToStart = tangentLine(startPose, humanZone, false);
        toLine2Finish = tangentLine(humanZone, humanZoneShot, true);
        line2ToShot = tangentLine(humanZoneShot, humanZoneShot, false);
        shotToGate = tangentCurve(
                humanZoneShot,
                new Pose(46.915, 36.863),
                leftLine,
                false
        );
        gateToShot = tangentLine(leftLine, leftLineShot, true);
        shotToGateAgain = tangentCurve(
                leftLineShot,
                new Pose(24.327, 27.003),
                new Pose(23.774, 33.830),
                middleLine,
                false
        );
        openGate = constantHeadingLine(
                middleLine,
                openGatePose,
                Math.toRadians(90),
                false
        );
        gateToFinalShot = linearHeadingCurve(
                openGatePose,
                new Pose(48.775, 68.616),
                middleLineShot,
                Math.toRadians(90),
                Math.toRadians(180),
                false
        );
        shotToLine1Start = linearHeadingCurve(
                middleLineShot,
                new Pose(39.924, 53.095),
                gatePose,
                Math.toRadians(180),
                Math.toRadians(150),
                false
        );
        line1StartToFinish = linearHeadingCurve(
                gatePose,
                new Pose(52.684, 67.132),
                gateShot,
                Math.toRadians(150),
                Math.toRadians(180),
                false
        );
        intakeRightLine = tangentLine(gateShot, rightLine, false);
        line1ToFinalShot = tangentLine(rightLine, finalShot, true);
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

    private PathChain tangentCurve(Pose blueStart,
                                   Pose blueControl1,
                                   Pose blueControl2,
                                   Pose blueEnd,
                                   boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose control1 = paths().pose(blueControl1);
        Pose control2 = paths().pose(blueControl2);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierCurve(start, control1, control2, end))
                .setTangentHeadingInterpolation();

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain middleLineGatePath(Pose blueStart, Pose blueMiddleLine, Pose blueGateOpen) {
        Pose start = paths().pose(blueStart);
        Pose control1 = paths().pose(new Pose(24.327, 27.003));
        Pose control2 = paths().pose(new Pose(23.774, 33.830));
        Pose middleLine = paths().pose(blueMiddleLine);
        Pose gateOpen = paths().pose(blueGateOpen);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierCurve(start, control1, control2, middleLine))
                .addPath(new BezierLine(middleLine, gateOpen))
                .setConstantHeadingInterpolation(allianceHeading(Math.toRadians(90)));

        return builder.build();
    }

    private PathChain finalRightLinePath(Pose blueStart, Pose blueRightLine, Pose blueFinalShot) {
        Pose start = paths().pose(blueStart);
        Pose rightLine = paths().pose(blueRightLine);
        Pose finalShot = paths().pose(blueFinalShot);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierLine(start, rightLine))
                .addPath(new BezierLine(rightLine, finalShot))
                .setTangentHeadingInterpolation()
                .setReversed();

        return builder.build();
    }

    private PathChain linearHeadingLine(Pose blueStart,
                                        Pose blueEnd,
                                        double blueStartHeading,
                                        double blueEndHeading,
                                        boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(
                        allianceHeading(blueStartHeading),
                        allianceHeading(blueEndHeading)
                );

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain constantHeadingLine(Pose blueStart, Pose blueEnd, double blueHeading, boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierLine(start, end))
                .setConstantHeadingInterpolation(allianceHeading(blueHeading));

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain linearHeadingCurve(Pose blueStart,
                                         Pose blueControl,
                                         Pose blueEnd,
                                         double blueStartHeading,
                                         double blueEndHeading,
                                         boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose control = paths().pose(blueControl);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(
                        allianceHeading(blueStartHeading),
                        allianceHeading(blueEndHeading)
                );

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private double allianceHeading(double blueHeading) {
        return alliance() == AutoAlliance.BLUE ? blueHeading : Math.PI - blueHeading;
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
                .followAsync(line2ControlToStart, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(toLine2Finish, LINE_INTAKE_SPEED, false)
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
                .followAsync(openGate, 1.0, false)
                .waitForFollowerIdle()
                .doAction(Actions.waitSeconds(GATE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .followAsync(gateToFinalShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .followAsync(shotToLine1Start, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(line1StartToFinish, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .doAction(stopIntake())
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .followAsync(intakeRightLine, 1.0, false)
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

    public static class Blue extends Unsorted15BallAuto {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.BLUE;
        }
    }

    public static class Red extends Unsorted15BallAuto {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.RED;
        }
    }
}
