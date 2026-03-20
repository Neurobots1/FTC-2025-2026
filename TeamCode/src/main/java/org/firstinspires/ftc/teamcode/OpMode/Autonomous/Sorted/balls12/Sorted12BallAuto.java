package org.firstinspires.ftc.teamcode.OpMode.Autonomous.Sorted.balls12;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpMode.Autonomous.templates.BaseSortedAutoTemplate;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.ModularAutoBuilder;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.SortPattern;

public abstract class Sorted12BallAuto extends BaseSortedAutoTemplate {

    private PathChain toPreloadShot;
    private PathChain toLine1Start;
    private PathChain toLine1Finish;
    private PathChain line1ToShot;
    private PathChain toLine2Start;
    private PathChain toLine2Finish;
    private PathChain line2ToFinalShot;

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
        Pose line2Finish = new Pose(15, 55, Math.toRadians(180));
        Pose line2Control = new Pose(45, 55);
        Pose finalShot = new Pose(55, 105, Math.toRadians(145));

        toPreloadShot = paths().shotLine(startPose, preloadShot);
        toLine1Start = paths().line(preloadShot, line1Start);
        toLine1Finish = paths().line(line1Start, line1Finish);
        line1ToShot = paths().shotLine(line1Finish, preloadShot);
        toLine2Start = paths().line(preloadShot, line2Start);
        toLine2Finish = paths().line(line2Start, line2Finish);
        line2ToFinalShot = paths().shotCurve(line2Finish, line2Control, finalShot);
    }

    @Override
    protected void buildRoutine() {
        ModularAutoBuilder builder = new ModularAutoBuilder(robot());
        builder
                .readSortPattern(tagReader(), 1.0, SortPattern.NOSORT)
                .doAction(robot().enableSortedPreSpin(true))
                .followAsync(toPreloadShot, 1.0, true)
                .waitForSortedReadyToShoot()
                .doAction(robot().startSortedPreloadShot())
                .doAction(robot().waitForSortedIdle())
                .waitForFollowerIdle()
                .followAsync(toLine1Start, 1.0, true)
                .waitForFollowerIdle()
                .followAsync(toLine1Finish, 1.0, true)
                .waitForFollowerIdle()
                .doAction(robot().enableSortedPreSpin(true))
                .doAction(robot().startSortedIntake(1))
                .doAction(robot().waitForSortedIdle())
                .followAsync(line1ToShot, 1.0, true)
                .waitForSortedReadyToShoot()
                .doAction(robot().startSortedShot(1))
                .doAction(robot().waitForSortedIdle())
                .waitForFollowerIdle()
                .followAsync(toLine2Start, 1.0, true)
                .waitForFollowerIdle()
                .followAsync(toLine2Finish, 1.0, true)
                .waitForFollowerIdle()
                .doAction(robot().enableSortedPreSpin(true))
                .doAction(robot().startSortedIntake(2))
                .doAction(robot().waitForSortedIdle())
                .followAsync(line2ToFinalShot, 1.0, true)
                .waitForSortedReadyToShoot()
                .doAction(robot().startSortedShot(2))
                .doAction(robot().waitForSortedIdle())
                .waitForFollowerIdle()
                .stopSortedShooter();

        scheduler().setAction(builder.build());
    }

    public static class Blue extends Sorted12BallAuto {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.BLUE;
        }
    }

    public static class Red extends Sorted12BallAuto {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.RED;
        }
    }
}
