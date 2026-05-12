package org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.AutoAction;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Actions;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.SortPattern;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.LimelightTagReader;

import java.util.ArrayList;
import java.util.List;

public class ModularAutoBuilder {

    private final AutoRobotFacade robot;
    private final List<AutoAction> steps = new ArrayList<>();

    public ModularAutoBuilder(AutoRobotFacade robot) {
        this.robot = robot;
    }

    public ModularAutoBuilder doAction(AutoAction action) {
        steps.add(action);
        return this;
    }

    public ModularAutoBuilder follow(PathChain path, double speed, boolean holdEnd) {
        steps.add(Actions.followPath(robot.follower(), path, speed, holdEnd));
        return this;
    }

    public ModularAutoBuilder followAsync(PathChain path, double speed, boolean holdEnd) {
        steps.add(Actions.instant(() -> robot.follower().followPath(path, speed, holdEnd)));
        return this;
    }

    public ModularAutoBuilder waitForFollowerIdle() {
        steps.add(robot.waitForFollowerIdle());
        return this;
    }

    public ModularAutoBuilder waitSeconds(double seconds) {
        steps.add(Actions.waitSeconds(seconds));
        return this;
    }

    public ModularAutoBuilder waitForSortedShotZone() {
        steps.add(robot.waitForSortedShotZone());
        return this;
    }

    public ModularAutoBuilder waitForSortedReadyToShoot() {
        steps.add(robot.waitForSortedReadyToShoot());
        return this;
    }

    public ModularAutoBuilder waitForUnsortedShotZone() {
        steps.add(robot.waitForUnsortedShotZone());
        return this;
    }

    public ModularAutoBuilder waitForUnsortedReadyToShoot() {
        steps.add(robot.waitForUnsortedReadyToShoot());
        return this;
    }

    public ModularAutoBuilder lockSortPattern(LimelightTagReader tagReader, double timeoutSeconds, SortPattern fallbackPattern) {
        steps.add(robot.lockSortPattern(tagReader, timeoutSeconds, fallbackPattern));
        return this;
    }

    public ModularAutoBuilder readSortPattern(LimelightTagReader tagReader, double timeoutSeconds, SortPattern fallbackPattern) {
        return lockSortPattern(tagReader, timeoutSeconds, fallbackPattern);
    }

    public ModularAutoBuilder shootPreloadSorted(PathChain toShotPose) {
        steps.add(robot.enableSortedPreSpin(true));
        steps.add(Actions.followPath(robot.follower(), toShotPose, 1.0, true));
        steps.add(robot.startSortedPreloadShot());
        steps.add(robot.waitForSortedIdle());
        return this;
    }

    public ModularAutoBuilder shootSortedPreload(PathChain toShotPose) {
        return shootPreloadSorted(toShotPose);
    }

    public ModularAutoBuilder collectSortedLine(int line, PathChain toLineStart, PathChain toLineFinish) {
        return collectSortedLine(line, toLineStart, toLineFinish, 1.0);
    }

    public ModularAutoBuilder collectSortedLine(int line, PathChain toLineStart, PathChain toLineFinish, double finishSpeed) {
        steps.add(robot.enableSortedPreSpin(true));
        steps.add(robot.startSortedIntake(line));
        steps.add(Actions.followPath(robot.follower(), toLineStart, 1.0, true));
        steps.add(Actions.followPath(robot.follower(), toLineFinish, finishSpeed, true));
        steps.add(robot.waitForSortedIdle());
        return this;
    }

    public ModularAutoBuilder shootSortedLine(int line, PathChain toShotPose) {
        steps.add(robot.enableSortedPreSpin(true));
        steps.add(Actions.followPath(robot.follower(), toShotPose, 1.0, true));
        steps.add(robot.startSortedShot(line));
        steps.add(robot.waitForSortedIdle());
        return this;
    }

    public ModularAutoBuilder sortedCycle(int line,
                                          PathChain toLineStart,
                                          PathChain toLineFinish,
                                          double finishSpeed,
                                          PathChain toShotPose) {
        return collectSortedLine(line, toLineStart, toLineFinish, finishSpeed)
                .shootSortedLine(line, toShotPose);
    }

    public ModularAutoBuilder sortedCycle(int line,
                                          PathChain toLineStart,
                                          PathChain toLineFinish,
                                          PathChain toShotPose) {
        return sortedCycle(line, toLineStart, toLineFinish, 1.0, toShotPose);
    }

    public ModularAutoBuilder stopSortedShooter() {
        steps.add(robot.stopSortedShooter());
        return this;
    }

    public ModularAutoBuilder startSortedIntake(int line) {
        steps.add(robot.startSortedIntake(line));
        return this;
    }

    public ModularAutoBuilder startSortedShot(int line) {
        steps.add(robot.startSortedShot(line));
        return this;
    }

    public ModularAutoBuilder waitForSortedIdle() {
        steps.add(robot.waitForSortedIdle());
        return this;
    }

    public ModularAutoBuilder shootUnsorted(PathChain toShotPose) {
        steps.add(robot.enableUnsortedShooter(true));
        steps.add(Actions.followPath(robot.follower(), toShotPose, 1.0, true));
        steps.add(robot.startUnsortedShot());
        steps.add(robot.waitForUnsortedShotDone());
        return this;
    }

    public ModularAutoBuilder enableUnsortedShooter(boolean enabled) {
        steps.add(robot.enableUnsortedShooter(enabled));
        return this;
    }

    public ModularAutoBuilder startUnsortedShot() {
        steps.add(robot.startUnsortedShot());
        return this;
    }

    public ModularAutoBuilder waitForUnsortedShotDone() {
        steps.add(robot.waitForUnsortedShotDone());
        return this;
    }

    public ModularAutoBuilder stopUnsortedShooter() {
        steps.add(robot.enableUnsortedShooter(false));
        return this;
    }

    public AutoAction build() {
        return Actions.sequence(steps.toArray(new AutoAction[0]));
    }
}
