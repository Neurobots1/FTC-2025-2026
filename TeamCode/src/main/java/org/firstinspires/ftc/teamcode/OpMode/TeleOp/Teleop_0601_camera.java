package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Launcher23511;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.Relocalisation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.List;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.ConvertToPedroPose;

@TeleOp
public class Teleop_0601_camera extends OpMode {

    public static boolean usePIDF = true;
    public static boolean shooterEnabled = false;
    public static double targetTicksPerSecond = 0;
    public static double testPower = 1.0;
    public ConvertToPedroPose convertToPedroPose;

    private JoinedTelemetry jt;
    private Follower follower;
    private DcMotorEx intake;
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(90));

    private static final double GOAL_X = 132;
    private static final double GOAL_Y = 132;
    private final Pose goalPose = new Pose(12, 132, 0.0);
    private Teleopblue1214_N_debug currentPosition;
    private Launcher23511 launcher;
    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;
    private VoltageSensor voltageSensor;
    private TelemetryManager telemetryManager;
    private IntakeMotor intkM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private Robot init;

    private Relocalisation relocalisation;
    private AprilTagPipeline aprilTagPipeline;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        jt = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        flywheelMotorOne = hardwareMap.get(DcMotorEx.class, "ShooterA");
        flywheelMotorTwo = hardwareMap.get(DcMotorEx.class, "ShooterB");
        intkM = new IntakeMotor(hardwareMap);
        init = new Robot(hardwareMap);
        currentPosition = new Teleopblue1214_N_debug();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcher = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        launcher.init();

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        convertToPedroPose = new ConvertToPedroPose();


        aprilTagPipeline = new AprilTagPipeline(hardwareMap);
        aprilTagPipeline.startCamera();
        relocalisation = new Relocalisation(hardwareMap, aprilTagPipeline);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(startingPose);
    }

    @Override
    public void loop() {
        follower.update();


        if (!slowMode) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false, 0
            );
        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier
            );
        }

        if (gamepad1.right_bumper) intkM.intake();
        else if (gamepad1.left_bumper) intkM.outtake();
        else intkM.stop();


        int tagCount = -1;
        List<AprilTagDetection> dets = aprilTagPipeline.getAllDetections();
        if (dets != null) {
            tagCount = dets.size();
        }
        telemetry.addData("Tag count", tagCount);
        jt.addData("Current Position", follower.getPose());

        if (gamepad1.a) {
            Pose tagPose = relocalisation.relocalisation();

            if (tagPose != null) {
                Pose pedroPose = ConvertToPedroPose.convertToPedroPose(tagPose);
                if (pedroPose != null) {
                    follower.setPose(pedroPose);
                } else {
                    telemetryManager.debug("RelocState", "NO TAG / INVALID");
                }
            } else {
                telemetryManager.debug("RelocState", "IDLE");
            }

            telemetryManager.debug("shooterEnabled", shooterEnabled);
            telemetryManager.debug("usePIDF", usePIDF);
            telemetryManager.debug("targetTicksPerSecond", targetTicksPerSecond);
            telemetryManager.debug("testPower", testPower);
            telemetryManager.debug("P", Launcher23511.P);
            telemetryManager.debug("I", Launcher23511.I);
            telemetryManager.debug("D", Launcher23511.D);
            telemetryManager.debug("F", Launcher23511.F);
            telemetryManager.debug("NOMINAL_VOLTAGE", Launcher23511.NOMINAL_VOLTAGE);
            telemetryManager.debug("pose2D", follower.getPose());

            jt.addData("targetTicksPerSecond", "%.0f", targetTicksPerSecond);
            jt.addData("Current Position", follower.getPose());
            jt.addData("Power", intake.getPower());
            jt.update();
            telemetryManager.update();

            telemetry.update();
        }
    }
}
