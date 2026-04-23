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
    public static double GATE_INTAKE_SECONDS = 1.5;
    public static double LINE_INTAKE_SPEED = 0.5;

    private PathChain toPreloadShot;
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
        Pose gatePose = AutoPoseConstants.gatePose();
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
                    gatePose,
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
                gatePose,
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
                                               Pose gatePose,
                                               Pose gateControl,
                                               Pose line1Start,
                                               Pose line1Finish,
                                               Pose finalShot,
                                               Pose gateShootControl) {
        Pose preloadShotToLine2Heading = new Pose(
                preloadShot.getX(),
                preloadShot.getY(),
                line2Start.getHeading()
        );
        Pose preloadShotToGateHeading = new Pose(
                preloadShot.getX(),
                preloadShot.getY(),
                gatePose.getHeading()
        );
        Pose finalShotToLine1Heading = new Pose(
                finalShot.getX(),
                finalShot.getY(),
                line1Start.getHeading()
        );

        toPreloadShot = paths().shotLine(startPose, preloadShotToLine2Heading);
        toLine2Start = paths().curve(preloadShotToLine2Heading, line2Control, line2Start);
        toLine2Finish = paths().line(line2Start, line2Finish);
        line2ToShot = paths().curve(line2Finish, gateShootControl, preloadShotToGateHeading);
        shotToGate = paths().curve(preloadShotToGateHeading, gateControl, gatePose);
        gateToShot = paths().curve(gatePose, gateShootControl, preloadShotToGateHeading);
        shotToGateAgain = paths().curve(preloadShotToGateHeading, gateControl, gatePose);
        gateToFinalShot = paths().curve(gatePose, gateShootControl, finalShotToLine1Heading);
        shotToLine1Start = paths().line(finalShotToLine1Heading, line1Start);
        line1StartToFinish = paths().line(line1Start, line1Finish);
        line1ToFinalShot = paths().shotLine(line1Finish, finalShot);
    }

    private void buildNormalPaths(Pose startPose,
                                  Pose preloadShot,
                                  Pose line2Start,
                                  Pose line2Finish,
                                  Pose line2Control,
                                  Pose gatePose,
                                  Pose gateControl,
                                  Pose line1Start,
                                  Pose line1Finish,
                                  Pose finalShot,
                                  Pose gateShootControl) {
        toPreloadShot = paths().shotLine(startPose, preloadShot);
        toLine2Start = paths().curve(preloadShot, line2Control, line2Start);
        toLine2Finish = paths().line(line2Start, line2Finish);
        line2ToShot = paths().shotCurve(line2Finish, gateShootControl, preloadShot);
        shotToGate = paths().curve(preloadShot, gateControl, gatePose);
        gateToShot = paths().shotCurve(gatePose, gateShootControl, preloadShot);
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
                .followAsync(toLine2Start, 1.0, false)
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
