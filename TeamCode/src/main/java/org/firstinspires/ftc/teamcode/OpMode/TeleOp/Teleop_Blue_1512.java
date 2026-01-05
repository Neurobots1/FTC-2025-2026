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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.ConvertToPedroPose;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.Relocalisationfilter;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipelinetele;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;

import java.util.List;

@TeleOp
public class Teleop_Blue_1512 extends OpMode {

    public static boolean usePIDF = true;
    public static boolean shooterEnabled = false;
    public static double targetTicksPerSecond = 0;
    public static double testPower = 1.0;
    private JoinedTelemetry jt;
    private Relocalisationfilter relocalisationfilter;
    private AprilTagPipeline aprilTagPipeline;
    private Follower follower;
    private DcMotorEx intake;
    private final Pose startingPose = new Pose(72,72,Math.toRadians(90));
    private ConvertToPedroPose convertToPedroPose;
    private static final double GOAL_X = 132;
    private static final double GOAL_Y = 132;
    private final Pose goalPose = new Pose(12, 132, 0.0);
    private Launcher23511 launcher;
    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;
    private VoltageSensor voltageSensor;
    private TelemetryManager telemetryManager;
    private IntakeMotor intkM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private Robot init;




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
        aprilTagPipeline = new AprilTagPipeline();
        convertToPedroPose = new ConvertToPedroPose();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcher = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        launcher.init();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(startingPose);
    }

    @Override
    public void loop(){

        follower.update();

        List<AprilTagDetection> detections = aprilTagPipeline.getAllDetections();    double headingInput =
                -gamepad1.right_stick_x;

        if (!slowMode)follower.setTeleOpDrive(
                - gamepad1.left_stick_y,
                - gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false, 3.142 // Doit etre a 0 pour rouge , mais pour bleu c'est 3.142 radian ( 180 degree)


        );

        else follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier
        );

        if (gamepad1.right_bumper) intkM.intake();
        else if (gamepad1.left_bumper) intkM.outtake();
        else intkM.stop();

        if(gamepad1.start) relocalisationfilter.relocalisation();
        convertToPedroPose.convertToPedroPose(relocalisationfilter.filteredPose.getPose());

        if (detections.isEmpty()){
            telemetry.addLine("No AprilTags Detected");
        } else {
            telemetry.addData("Detections", detections.size());
            for (AprilTagDetection detection : detections) {
                telemetry.addLine(String.format("ID: %d | ftcpose: (%.2f, %.2f)",
                        detection.id,
                        detection.ftcPose.x,
                        detection.ftcPose.y));
            }
        }





        if (gamepad1.a) shooterEnabled = true;
        if (gamepad1.b) shooterEnabled = false;



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
        jt.addData("PedroPose",relocalisationfilter.relocalisation());
        jt.addData("positon", follower.getPose());
        jt.addData("Power", intake.getPower());
        jt.update();
        telemetryManager.update();

    }

}
