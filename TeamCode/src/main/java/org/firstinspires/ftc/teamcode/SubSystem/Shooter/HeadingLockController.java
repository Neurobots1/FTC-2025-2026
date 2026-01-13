package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

public class HeadingLockController {

    public static final double GOAL_X = 12;
    public static final double GOAL_Y = 132;

    private final PIDFController headingController;
    private final Follower follower;
    private double headingGoal = 0.0;

    public HeadingLockController(PIDFController headingController, Follower follower) {
        this.headingController = headingController;
        this.follower = follower;
    }

    private double calculateTargetHeadingToGoal(Pose pose) {
        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();
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

        headingController.setCoefficients(follower.constants.coefficientsHeadingPIDF);

        headingGoal = calculateTargetHeadingToGoal(pose);
        double error = getHeadingError(pose);
        headingController.updateError(error);
        return headingController.run();
    }

    public double getHeadingGoal() {
        return headingGoal;
    }
}
