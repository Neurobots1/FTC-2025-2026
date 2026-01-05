package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Auto_Shoot {

    private Follower follower;

    public static double GOAL_X = 132.0;
    public static double GOAL_Y = 132.0;
    public static double GOAL_HEIGHT = 48.0;

    private double distanceToGoal = 0.0;
    private double angleToGoal = 0.0;
    private boolean targetInRange = false;

    private DcMotorEx shooterMotorA;
    private DcMotorEx shooterMotorB;
    private VoltageSensor voltageSensor;

    private double targetRPM = 0.0;
    private double currentRPM = 0.0;

    public static double P = 0.0001;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.000185;
    public static double NOMINAL_VOLTAGE = 12.0;

    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_TOLERANCE = 50.0;

    private static final double[][] RPM_CURVE = {
            {12.0, 2000},
            {24.0, 2200},
            {36.0, 2400},
            {48.0, 2600},
            {60.0, 2800},
            {72.0, 3000},
            {84.0, 3200},
            {96.0, 3400},
            {108.0, 3600},
            {120.0, 3800}
    };

    public static double MIN_RPM = 1500;
    public static double MAX_RPM = 4000;
    public static double MIN_DISTANCE = 10.0;
    public static double MAX_DISTANCE = 150.0;

    // si millimètres, utiliser 0.0393701
    // si pouces, utiliser 1.0
    public static double PEDRO_TO_INCHES = 0.0393701;

    private boolean autoAdjustEnabled = true;

    public Auto_Shoot(Follower follower, DcMotorEx motorA, DcMotorEx motorB, VoltageSensor voltage) {
        this.follower = follower;
        shooterMotorA = motorA;
        shooterMotorB = motorB;
        voltageSensor = voltage;

        shooterMotorA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotorA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotorB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterMotorA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void update() {
        follower.update();

        if (autoAdjustEnabled) {
            calculateDistanceToGoal();

            if (targetInRange) {
                targetRPM = calculateRPMFromDistance(distanceToGoal);
            }
        }

        updateLauncher();
    }

    private void calculateDistanceToGoal() {
        Pose currentPose = follower.getPose();

        double robotX = currentPose.getX();
        double robotY = currentPose.getY();

        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;

        double distancePedroUnits = Math.sqrt(deltaX * deltaX + deltaY * deltaY);


        distanceToGoal = distancePedroUnits * PEDRO_TO_INCHES;

        angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        targetInRange = (distanceToGoal >= MIN_DISTANCE && distanceToGoal <= MAX_DISTANCE);
    }

    private void updateLauncher() {
        if (targetRPM > 0) {
            double velocityA = shooterMotorA.getVelocity();
            double velocityB = shooterMotorB.getVelocity();
            currentRPM = ((velocityA + velocityB) / 2.0) * (60.0 / TICKS_PER_REV);

            double voltage = voltageSensor.getVoltage();
            double voltageCompensation = NOMINAL_VOLTAGE / voltage;

            double error = targetRPM - currentRPM;
            double feedforward = F * targetRPM * voltageCompensation;
            double proportional = P * error;

            double power = feedforward + proportional;
            power = Math.max(-1.0, Math.min(1.0, power));

            shooterMotorA.setPower(power);
            shooterMotorB.setPower(power);
        } else {
            shooterMotorA.setPower(0);
            shooterMotorB.setPower(0);
        }
    }

    private double calculateRPMFromDistance(double dist) {
        if (dist < MIN_DISTANCE) dist = MIN_DISTANCE;
        if (dist > MAX_DISTANCE) dist = MAX_DISTANCE;

        for (int i = 0; i < RPM_CURVE.length - 1; i++) {
            double dist1 = RPM_CURVE[i][0];
            double rpm1 = RPM_CURVE[i][1];
            double dist2 = RPM_CURVE[i + 1][0];
            double rpm2 = RPM_CURVE[i + 1][1];

            if (dist >= dist1 && dist <= dist2) {
                double ratio = (dist - dist1) / (dist2 - dist1);
                double interpolatedRPM = rpm1 + ratio * (rpm2 - rpm1);
                return Math.max(MIN_RPM, Math.min(MAX_RPM, interpolatedRPM));
            }
        }

        if (dist < RPM_CURVE[0][0]) {
            return RPM_CURVE[0][1];
        } else {
            return RPM_CURVE[RPM_CURVE.length - 1][1];
        }
    }

    public double calculateRPMWithHeight(double horizontalDistance) {
        double baseRPM = calculateRPMFromDistance(horizontalDistance);
        double heightFactor = 1.0 + (GOAL_HEIGHT / 100.0) * 0.1;
        return baseRPM * heightFactor;
    }

    public void shootAuto() {
        if (targetInRange && isAtTargetRPM()) {
            // Prêt à tirer
        }
    }

    public void shootManual(double rpm) {
        autoAdjustEnabled = false;
        targetRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
    }

    public void enableAutoMode() {
        autoAdjustEnabled = true;
    }

    public void stop() {
        targetRPM = 0;
        shooterMotorA.setPower(0);
        shooterMotorB.setPower(0);
    }

    public double getAngleToGoal() {
        Pose currentPose = follower.getPose();
        double currentHeading = Math.toDegrees(currentPose.getHeading());

        double targetAngle = angleToGoal;
        double angleDifference = targetAngle - currentHeading;

        while (angleDifference > 180) angleDifference -= 360;
        while (angleDifference < -180) angleDifference += 360;

        return angleDifference;
    }

    public boolean isFacingGoal(double toleranceDegrees) {
        return Math.abs(getAngleToGoal()) <= toleranceDegrees;
    }

    public Pose createPoseTowardsGoal() {
        Pose currentPose = follower.getPose();
        double headingToGoal = Math.toRadians(angleToGoal);
        return new Pose(currentPose.getX(), currentPose.getY(), headingToGoal);
    }

    // Getters
    public boolean isAtTargetRPM() {
        return Math.abs(currentRPM - targetRPM) < RPM_TOLERANCE;
    }

    public boolean isTargetInRange() {
        return targetInRange;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public double getAngleToGoalDegrees() {
        return angleToGoal;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public void setAutoAdjustEnabled(boolean enabled) {
        autoAdjustEnabled = enabled;
    }

    public boolean isAutoAdjustEnabled() {
        return autoAdjustEnabled;
    }

    public void setGoalPosition(double x, double y) {
        GOAL_X = x;
        GOAL_Y = y;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
    }

    public String getTargetInfo() {
        if (!targetInRange) {
            return "OUT OF RANGE";
        }
        return String.format("Target IN RANGE | %.1f\" → %.0f RPM", distanceToGoal, targetRPM);
    }

    public String getPositionInfo() {
        Pose pose = follower.getPose();
        return String.format("Robot: (%.1f, %.1f) | Goal: (%.1f, %.1f) | Dist: %.1f\"",
                pose.getX(), pose.getY(), GOAL_X, GOAL_Y, distanceToGoal);
    }
}





//.