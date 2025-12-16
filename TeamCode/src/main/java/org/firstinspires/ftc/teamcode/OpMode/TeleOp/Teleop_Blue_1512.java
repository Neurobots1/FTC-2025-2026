package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import static org.firstinspires.ftc.teamcode.OpMode.TeleOp.ExempleteleopBaron.startingPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.util.InterpLUT;
import org.firstinspires.ftc.teamcode.OpMode.Autonomous.Auto_1st_meet_Blue;
import org.firstinspires.ftc.teamcode.SubSystem.Shoot;
import org.firstinspires.ftc.teamcode.SubSystem.PIDController;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Robot;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Launcher23511;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.Relocalisationfilter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LUTs;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.ConvertToPedroPose;

@Configurable
@TeleOp (name = "Teleop_Blue_1512", group = "Tuning")
public class Teleop_Blue_1512 extends OpMode {

    public TelemetryManager telemetryManager;
    public PathChain pathChain;
    public static boolean usePIDF = true;
    public static boolean shooterEnabled = false;
    public static double targetTicksPerSecond = 0;
    public static double testPower = 1.0;
    private JoinedTelemetry jt;
    private Relocalisationfilter relocalisationfilter;
    private Follower follower;
    private DcMotorEx intake;
    public final Pose startingPose = new Pose(72,72,Math.toRadians(90));
    private ConvertToPedroPose convertToPedroPose;
    private static final double GOAL_X = 12;
    private static final double GOAL_Y = 132;
    private final Pose goalPose = new Pose(12, 132, 0.0);
    // 🔹 This stays the same, but now uses Hermite inside
    private teleopblue.ShooterTicksLUT shooterLUT = new teleopblue.ShooterTicksLUT();

    private final InterpLUT lut = new InterpLUT();

    private Launcher23511 launcher;
    private DcMotorEx flywheelMotorOne;
    private DcMotorEx flywheelMotorTwo;
    private VoltageSensor voltageSensor;
    private IntakeMotor intkM;
    private Robot init;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;



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
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcher = new Launcher23511(flywheelMotorOne, flywheelMotorTwo, voltageSensor);
        launcher.init();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();


    }
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(startingPose);
    }

    public void loop() {
        follower.update();

        double headingInput =
                 -gamepad1.right_stick_x;

        if (!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
               false, 3.142 // Doit etre a 0 pour rouge, mais 3.124 pour bleu (EN RADIANT = 180 degrees)

        );


        else follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier
        );

if (gamepad1.right_bumper) intkM.intake();
else if (gamepad1.left_bumper) intkM.slowOuttake();
else intkM.stop();

if (gamepad1.a) shooterEnabled = true;
if (gamepad1.b) shooterEnabled = true;
}


}

