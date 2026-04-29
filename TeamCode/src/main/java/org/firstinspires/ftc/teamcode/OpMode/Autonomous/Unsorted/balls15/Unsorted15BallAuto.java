package org.firstinspires.ftc.teamcode.OpMode.Autonomous.Unsorted.balls15;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Constants.AutoPoseConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterHardwareConstants;
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
    private PathChain gateToFinalShot;
    private PathChain shotToLine1Start;
    private PathChain line1StartToFinish;
    private PathChain line1ToFinalShot;

    @Override
    protected Pose startPose() {
        return paths().pose(AutoPoseConstants.CloseStartPose());
    }

    @Override
    protected void buildPaths() {
        Pose startPose = AutoPoseConstants.CloseStartPose();
        Pose preloadShot = AutoPoseConstants.CloseShootPose();
        Pose line2Start = AutoPoseConstants.line2StartPose();
        Pose line2Finish = AutoPoseConstants.line2FinishPoseUnsorted();
        Pose line2Control = AutoPoseConstants.line2ControlPose();
        Pose line2ReturnControl = AutoPoseConstants.line2ReturnControlPose();
        Pose gatePose = AutoPoseConstants.gatePose();
        Pose firstGatePose = AutoPoseConstants.firstGatePose();
        Pose gateControl = AutoPoseConstants.gateControlPose();
        Pose line1Start = AutoPoseConstants.line1StartPose();
        Pose line1Finish = AutoPoseConstants.line1FinishPose();
        Pose finalShot = AutoPoseConstants.finalShotPose();
        Pose gateShootControl = AutoPoseConstants.gateShootControl();

        if (ShooterHardwareConstants.turretEnabled) {
            buildTurretEnabledNormalPaths(
                    startPose,
                    preloadShot,
                    line2Start,
                    line2Finish,
                    line2Control,
                    line2ReturnControl,
                    gatePose,
                    firstGatePose,
                    gateControl,
                    line1Start,
                    line1Finish,
                    finalShot,
                    gateShootControl
            );
            return;
        }

        buildNormalPaths(
                startPose,
                preloadShot,
                line2Start,
                line2Finish,
                line2Control,
                line2ReturnControl,
                gatePose,
                firstGatePose,
                gateControl,
                line1Start,
                line1Finish,
                finalShot,
                gateShootControl
        );
    }

    private void buildTurretEnabledNormalPaths(Pose startPose,
                                               Pose preloadShot,
                                               Pose line2Start,
                                               Pose line2Finish,
                                               Pose line2Control,
                                               Pose line2ReturnControl,
                                               Pose gatePose,
                                               Pose firstGatePose,
                                               Pose gateControl,
                                               Pose line1Start,
                                               Pose line1Finish,
                                               Pose finalShot,
                                               Pose gateShootControl) {
        Pose preloadShotHeading = new Pose(
                preloadShot.getX(),
                preloadShot.getY(),
                Math.toRadians(180)
        );
        Pose finalShotHeading = new Pose(
                finalShot.getX(),
                finalShot.getY(),
                Math.toRadians(180)
        );

        toPreloadShot = paths().line(startPose, preloadShotHeading);
        toLine2Control = paths().tangentLine(preloadShotHeading, line2Control);
        line2ControlToStart = paths().tangentLine(line2Control, line2Start);
        toLine2Finish = paths().line(line2Start, line2Finish);
        line2ToShot = paths().curve(line2Finish, line2ReturnControl, preloadShotHeading);
        shotToGate = paths().curve(preloadShotHeading, gateControl, firstGatePose);
        gateToShot = paths().curve(firstGatePose, gateShootControl, preloadShotHeading);
        shotToGateAgain = paths().curve(preloadShotHeading, gateControl, gatePose);
        gateToFinalShot = paths().curve(gatePose, gateShootControl, finalShotHeading);
        shotToLine1Start = paths().line(finalShotHeading, line1Start);
        line1StartToFinish = paths().line(line1Start, line1Finish);
        line1ToFinalShot = paths().line(line1Finish, finalShot);
    }

    private void buildNormalPaths(Pose startPose,
                                  Pose preloadShot,
                                  Pose line2Start,
                                  Pose line2Finish,
                                  Pose line2Control,
                                  Pose line2ReturnControl,
                                  Pose gatePose,
                                  Pose firstGatePose,
                                  Pose gateControl,
                                  Pose line1Start,
                                  Pose line1Finish,
                                  Pose finalShot,
                                  Pose gateShootControl) {
        toPreloadShot = paths().shotLine(startPose, preloadShot);
        toLine2Control = paths().tangentLine(preloadShot, line2Control);
        line2ControlToStart = paths().tangentLine(line2Control, line2Start);
        toLine2Finish = paths().line(line2Start, line2Finish);
        line2ToShot = paths().shotCurve(line2Finish, line2ReturnControl, preloadShot);
        shotToGate = paths().curve(preloadShot, gateControl, firstGatePose);
        gateToShot = paths().shotCurve(firstGatePose, gateShootControl, preloadShot);
        shotToGateAgain = paths().curve(preloadShot, gateControl, gatePose);
        gateToFinalShot = paths().shotCurve(gatePose, gateShootControl, preloadShot);
        shotToLine1Start = paths().line(finalShot, line1Start);
        line1StartToFinish = paths().line(line1Start, line1Finish);
        line1ToFinalShot = paths().shotLine(line1Finish, finalShot);
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
