package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.SubSystem.Auto_pathBuild_Blue;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.IndexerMode;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_GPP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PPG;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PGP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_NoSort;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

import javax.xml.namespace.QName;

@Autonomous (name = "Auto15Balles", group = "Examples")
public class Auto15Balles extends OpMode {

    private Follower follower;
    private Auto_pathBuild_Blue autoPathBuild;
    private RevColorSensorV3 colorSensor;
    private AprilTagPipeline aprilTag;

    private Indexer_Base indexer;
    private Indexer_PGP indexer_pgp;
    private Indexer_GPP indexer_gpp;
    private Indexer_PPG indexer_ppg;
    private Indexer_NoSort indexer_noSort;

    private IndexerMode activeIndexer;

    private IntakeMotor intkM;

    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;

    private VoltageSensor voltageSensor;
    private LauncherSubsystem Shooter;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private TelemetryManager telemetryM;

    private boolean useGPPMode = false;
    private boolean usePGPMode = false;
    private boolean usePPGMode = false;
    private boolean useNoSortMode = false;

    private boolean modeLocked = false;
    private int detectedAprilTagId = -1;




    public void init(){

    }
    public void stop(){

    }
    public void loop(){

    }
}
