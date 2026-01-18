package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;

@TeleOp(name="Tourelle Auto")
public class Automatic_Turret extends OpMode {

    private DcMotor moteurRotation;
    private Follower follower;
    private RevColorSensorV3 capteurTourelle;
    public Robot robot;

    public static double GOAL_X = 132.0;
    public static double GOAL_Y = 132.0;

    private double vitesseTourelle = 0.3;
    private double toleranceAngle = 2.0;
    private boolean zeroPoint = false;
    private int zeroPosition = 0;

    private static final double TICKS_PER_DEGREE = 10.0;

    @Override
    public void init() {
        // Initialisation du moteur
        moteurRotation = hardwareMap.get(DcMotor.class, "moteurTourelle");
        moteurRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moteurRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialisation du capteur
        capteurTourelle = hardwareMap.get(RevColorSensorV3.class, "capteurTourelle");

        if (robot != null && follower != null) {
            follower = follower;
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();
        }

        if (gamepad1.a) {
            TurretToGoal();
        } else if (gamepad1.b) {
            IsAtZeroPoint();
        } else {
            moteurRotation.setPower(0);
        }

        telemetry.addData("Position Moteur", moteurRotation.getCurrentPosition());
        telemetry.addData("Zero Point", zeroPoint);
        if (follower != null) {
            Pose currentPose = follower.getPose();
            telemetry.addData("Robot X", currentPose.getX());
            telemetry.addData("Robot Y", currentPose.getY());
            telemetry.addData("Robot Heading", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Angle vers Goal", calculateAngleToGoal());
            telemetry.addData("Angle Tourelle nécessaire", getRelativeTurretAngle());
        }
        telemetry.update();
    }


    public void IsAtZeroPoint() {
        if (capteurTourelle.getDistance(DistanceUnit.MM) <= 30) {
            moteurRotation.setPower(0);
            moteurRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            moteurRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            zeroPoint = true;
            zeroPosition = 0;
            telemetry.addData("IsZeroPoint", true);
        } else {
            moteurRotation.setPower(-0.3);
            zeroPoint = false;
        }
    }


    public void TurretToGoal() {
        if (follower == null) {
            telemetry.addData("Erreur", "Follower non initialisé");
            return;
        }

        if (!zeroPoint) {
            telemetry.addData("Avertissement", "Tourelle non calibrée. Appuyez sur B pour calibrer.");
            return;
        }

        double targetTurretAngle = getRelativeTurretAngle();

        int targetPosition = (int)(targetTurretAngle * TICKS_PER_DEGREE);

        int currentPosition = moteurRotation.getCurrentPosition();
        int error = targetPosition - currentPosition;
        double errorDegrees = error / TICKS_PER_DEGREE;

        if (Math.abs(errorDegrees) > toleranceAngle) {
            double power = Math.max(-vitesseTourelle, Math.min(vitesseTourelle, errorDegrees * 0.05));
            moteurRotation.setPower(power);
            telemetry.addData("Statut", "Ajustement en cours...");
        } else {
            moteurRotation.setPower(0);
            telemetry.addData("Statut", "Tourelle alignée!");
        }

        telemetry.addData("Erreur (degrés)", errorDegrees);
    }


    private double calculateAngleToGoal() {
        Pose currentPose = follower.getPose();

        double deltaX = GOAL_X - currentPose.getX();
        double deltaY = GOAL_Y - currentPose.getY();

        double angleToGoalRad = Math.atan2(deltaY, deltaX);
        double angleToGoalDeg = Math.toDegrees(angleToGoalRad);

        return angleToGoalDeg;
    }


    private double getRelativeTurretAngle() {
        Pose currentPose = follower.getPose();
        double currentHeading = Math.toDegrees(currentPose.getHeading());

        double targetAngle = calculateAngleToGoal();

        double relativeTurretAngle = targetAngle - currentHeading;

        while (relativeTurretAngle > 180) relativeTurretAngle -= 360;
        while (relativeTurretAngle < -180) relativeTurretAngle += 360;

        return relativeTurretAngle;
    }


    public double getAngleToGoal() {
        return getRelativeTurretAngle();
    }
}