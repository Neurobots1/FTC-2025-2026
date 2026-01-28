package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

public class HeadingLockController {

    public double BLUE_GOAL_X = 0;
    public double BLUE_GOAL_Y = 140;

    public double RED_GOAL_X = 140;
    public double RED_GOAL_Y = 140;

    private double goalX = BLUE_GOAL_X;
    private double goalY = BLUE_GOAL_Y;

    private final PIDFController headingController;
    private final Follower follower;
    private double headingGoal = 0.0;


    private static final PIDFCoefficients HEADING_PIDF =
            new PIDFCoefficients(0.6 ,0,0.02,0.02);

    public void setGoalY(double y) {
        this.goalY = y;
    }



    public HeadingLockController(PIDFController headingController, Follower follower) {
        this.headingController = headingController;
        this.follower = follower;
    }

    private double calculateTargetHeadingToGoal(Pose pose) {
        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();
        return Math.atan2(dy, dx);
    }

    private double getHeadingError(Pose pose) {
        double currentHeading = pose.getHeading();
        double smallestDiff = MathFunctions.getSmallestAngleDifference(currentHeading, headingGoal);
        double turnDir = MathFunctions.getTurnDirection(currentHeading, headingGoal);
        return turnDir * smallestDiff;
    }

    public double update(Pose pose, boolean lockEnabled, double manualTurn) {
        if (!lockEnabled) return manualTurn;

        headingController.setCoefficients(HEADING_PIDF);

        headingGoal = calculateTargetHeadingToGoal(pose);
        double error = getHeadingError(pose);
        headingController.updateError(error);
        return headingController.run();
    }

    public double getHeadingGoal() {
        return headingGoal;
    }

    public void setGoalBlue() {
        goalX = BLUE_GOAL_X;
        goalY = BLUE_GOAL_Y;
    }


    public void setGoalRed() {
        goalX = RED_GOAL_X;
        goalY = RED_GOAL_Y;
    }

    public double getGoalX() {
        return goalX;
    }

    public double getGoalY() {
        return goalY;
    }
}
