package org.firstinspires.ftc.teamcode.OpMode.Autonomous.Unsorted.balls12;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Constants.AutoPoseConstants;
import org.firstinspires.ftc.teamcode.OpMode.Autonomous.templates.BaseUnsortedAutoTemplate;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.AutoAction;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.ModularAutoBuilder;

public abstract class Unsorted12BallAuto extends BaseUnsortedAutoTemplate {
    public static double LINE_INTAKE_SECONDS = 0.9;
    public static double LINE_INTAKE_SPEED = 1.0;

    private PathChain toPreloadShot;
    private PathChain toLine1Start;
    private PathChain toLine1Finish;
    private PathChain line1ToShot;
    private PathChain toLine2Start;
    private PathChain toLine2Finish;
    private PathChain line2ToShot;
    private PathChain toLine3Start;
    private PathChain toLine3Finish;
    private PathChain line3ToFinalShot;

    @Override
    protected Pose startPose() {
        return paths().pose(AutoPoseConstants.CloseStartPose());
    }

    @Override
    protected void buildPaths() {
        Pose startPose = AutoPoseConstants.CloseStartPose();
        Pose preloadShot = AutoPoseConstants.CloseShootPose();
        Pose line1Start = AutoPoseConstants.line1StartPose();
        Pose line1Finish = AutoPoseConstants.line1FinishPose();
        Pose line2Start = AutoPoseConstants.line2StartPose();
        Pose line2Finish = AutoPoseConstants.line2FinishPoseUnsorted();
        Pose line2Control = AutoPoseConstants.line2ControlPose();
        Pose line3Start = AutoPoseConstants.line3StartPose();
        Pose line3Finish = AutoPoseConstants.line3FinishPose();
        Pose line3Control = AutoPoseConstants.line3ControlPose();
        Pose finalShot = AutoPoseConstants.finalShotPose();

        toPreloadShot = paths().shotLine(startPose, preloadShot);
        toLine1Start = paths().line(preloadShot, line1Start);
        toLine1Finish = paths().line(line1Start, line1Finish);
        line1ToShot = paths().shotLine(line1Finish, preloadShot);
        toLine2Start = paths().line(preloadShot, line2Start);
        toLine2Finish = paths().line(line2Start, line2Finish);
        line2ToShot = paths().shotCurve(line2Finish, line2Control, preloadShot);
        toLine3Start = paths().line(preloadShot, line3Start);
        toLine3Finish = paths().line(line3Start, line3Finish);
        line3ToFinalShot = paths().shotCurve(line3Finish, line3Control, finalShot);
    }

    @Override
    protected void buildRoutine() {
        ModularAutoBuilder builder = new ModularAutoBuilder(robot());
        builder
                .enableUnsortedShooter(true)
                .followAsync(toPreloadShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .followAsync(toLine1Start, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(toLine1Finish, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .doAction(Actions.waitSeconds(LINE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .followAsync(line1ToShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .followAsync(toLine2Start, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(toLine2Finish, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .doAction(Actions.waitSeconds(LINE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .followAsync(line2ToShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .followAsync(toLine3Start, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(toLine3Finish, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .doAction(Actions.waitSeconds(LINE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .followAsync(line3ToFinalShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .stopUnsortedShooter();

        scheduler().setAction(builder.build());
    }

    private AutoAction startTimedIntake() {
        return Actions.instant(() -> intake().intake());
    }

    private AutoAction stopIntake() {
        return Actions.instant(() -> intake().stop());
    }

    public static class Blue extends Unsorted12BallAuto {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.BLUE;
        }
    }

    public static class Red extends Unsorted12BallAuto {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.RED;
        }
    }
}
