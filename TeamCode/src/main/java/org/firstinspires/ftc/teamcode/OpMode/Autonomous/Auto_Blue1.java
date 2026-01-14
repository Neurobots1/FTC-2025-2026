package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Launcher23511;
import org.firstinspires.ftc.teamcode.SubSystem.Auto_pathBuild_Blue;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.SubSystem.AutoCase.Case_GPP;
import org.firstinspires.ftc.teamcode.SubSystem.AutoCase.Case_PGP;
import org.firstinspires.ftc.teamcode.SubSystem.AutoCase.Case_PPG;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Auto_blue1", group = "Examples")
public class Auto_Blue1 extends OpMode {

    private Follower follower;
    private Auto_pathBuild_Blue autoPathBuild;
    private AprilTagPipeline aprilTag;
    private Case_GPP caseGPP;
    private Case_PGP casePGP;
    private Case_PPG casePPG;

    private IntakeMotor intkM;
    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;

    public static boolean usePIDF = true;
    public static double targetTicksPerSecond = 725;
    public static double rawPower = -0.5;
    public static boolean rawPowerMode = false;

    private VoltageSensor voltageSensor;
    private Launcher23511 Shooter;
    public static boolean shooterEnabled = false;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private TelemetryManager telemetryM;


    private boolean useGPPMode = false;
    private boolean gppModeChecked = false;

    private boolean usePGPMode = false;
    private boolean pgpModeChecked = false;

    private boolean usePPGMode = false;
    private boolean ppgModeChecked = false;

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(autoPathBuild.TakePatern, 1, true);
                setPathState(1);

            case 1:


            if (!gppModeChecked && !useGPPMode) {
                int detectedID = getAprilTagID();
                if (detectedID == 21) {
                    useGPPMode = true;
                    telemetry.addData("Mode", "GPP Mode Activé - ID 21 détecté");
                }
                gppModeChecked = true;
            } else if (!pgpModeChecked && !usePGPMode) {
                int detectedID = getAprilTagID();
                if (detectedID == 22) {
                    usePGPMode = true;
                    telemetry.addData("Mode", "PGP Mode Activé - ID 22 détecté");
                }
                pgpModeChecked = true;
            } else if (!ppgModeChecked && !usePPGMode) {
                int detectedID = getAprilTagID();
                if (detectedID == 23) {
                    usePPGMode = true;
                    telemetry.addData("Mode", "PPG Mode Activé - ID 23 détecté");
                }
                ppgModeChecked = true;
            }


            if (useGPPMode) {
                caseGPP.autonomousPathUpdate();
                return;
            } else if (usePGPMode) {
                casePGP.autonomousPathUpdate();
                return;
            } else if (usePPGMode) {
                casePPG.autonomousPathUpdate();
                return;
            }

        }
    }

    // Méthode pour récupérer l'ID de l'AprilTag
    private int getAprilTagID() {
        List<AprilTagDetection> detections = aprilTag.getAllDetections();
        if (detections != null && !detections.isEmpty()) {
            return detections.get(0).id;
        }
        return -1; // Aucun tag détecté
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Télémétrie
        telemetry.addData("Mode GPP", useGPPMode ? "ACTIVÉ" : "Normal");
        telemetry.addData("AprilTag ID", getAprilTagID());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooterEnabled", shooterEnabled);
        telemetry.addData("usePIDF", usePIDF);
        telemetry.addData("targetTicksPerSecond", targetTicksPerSecond);
        telemetry.update();
    }

    @Override
    public void init() {
        setPathState(0);
        intkM = new IntakeMotor(hardwareMap);
        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Shooter = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        Shooter.setFlywheelTicks(725);
        shooterEnabled = false;
        Shooter.init();

        aprilTag = new AprilTagPipeline();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        autoPathBuild = new Auto_pathBuild_Blue(follower);
        follower.setStartingPose(autoPathBuild.startPose);
        autoPathBuild.buildPaths();

    }
}