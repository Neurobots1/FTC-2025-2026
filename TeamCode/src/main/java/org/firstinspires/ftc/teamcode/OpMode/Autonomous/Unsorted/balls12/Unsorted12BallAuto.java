package org.firstinspires.ftc.teamcode.OpMode.Autonomous.Unsorted.balls12;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

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
        return paths().pose(new Pose(20, 119, Math.toRadians(139)));
    }

    @Override
    protected void buildPaths() {
        Pose startPose = new Pose(20, 119, Math.toRadians(139));
        Pose preloadShot = new Pose(47, 92, Math.toRadians(139));
        Pose line1Start = new Pose(50, 88, Math.toRadians(190));
        Pose line1Finish = new Pose(23, 82, Math.toRadians(190));
        Pose line2Start = new Pose(50, 55, Math.toRadians(180));
        Pose line2Finish = new Pose(13, 55, Math.toRadians(180));
        Pose line2Control = new Pose(45, 55);
        // TODO: Confirm the actual blue-side start/finish coordinates for line 3.
        // These are a best-guess continuation below line 2 using the same driving pattern.
        Pose line3Start = new Pose(50, 26, Math.toRadians(180));
        Pose line3Finish = new Pose(13, 26, Math.toRadians(180));
        // TODO: Confirm the control point and shot pose for the line 3 return shot.
        Pose line3Control = new Pose(45, 26);
        Pose finalShot = new Pose(55, 105, Math.toRadians(145));

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
                .followAsync(toLine1Start, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(toLine1Finish, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .doAction(Actions.waitSeconds(LINE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .followAsync(line1ToShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .followAsync(toLine2Start, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(toLine2Finish, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
                .doAction(Actions.waitSeconds(LINE_INTAKE_SECONDS))
                .doAction(stopIntake())
                .followAsync(line2ToShot, 1.0, true)
                .waitForUnsortedReadyToShoot()
                .startUnsortedShot()
                .waitForUnsortedShotDone()
                .waitForFollowerIdle()
                .followAsync(toLine3Start, 1.0, false)
                .waitForFollowerIdle()
                .followAsync(toLine3Finish, LINE_INTAKE_SPEED, false)
                .waitForFollowerIdle()
                .doAction(startTimedIntake())
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
