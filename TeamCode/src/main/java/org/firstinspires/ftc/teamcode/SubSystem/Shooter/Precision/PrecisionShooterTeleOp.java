package org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.AllianceSelector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Precision Shooter TeleOp", group = "Shooter")
public class PrecisionShooterTeleOp extends OpMode {

    private final PrecisionShooterConfig config = new PrecisionShooterConfig();

    private Follower follower;
    private PrecisionShooterSubsystem shooter;
    private boolean autoAimEnabled = true;
    private boolean spinEnabled;
    private boolean prevA;
    private boolean prevX;
    private boolean prevY;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        shooter = PrecisionShooterSubsystem.create(hardwareMap, follower, config);
    }

    @Override
    public void start() {
        shooter.start();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true,
                AllianceSelector.Field.fieldCentricOffset(
                        shooter.getAlliance() == PrecisionShooterSubsystem.Alliance.BLUE
                                ? AllianceSelector.Alliance.BLUE
                                : AllianceSelector.Alliance.RED
                )
        );

        if (gamepad1.a && !prevA) {
            spinEnabled = !spinEnabled;
        }
        if (gamepad1.x && !prevX) {
            autoAimEnabled = !autoAimEnabled;
        }
        if (gamepad1.y && !prevY) {
            shooter.setAlliance(
                    shooter.getAlliance() == PrecisionShooterSubsystem.Alliance.BLUE
                            ? PrecisionShooterSubsystem.Alliance.RED
                            : PrecisionShooterSubsystem.Alliance.BLUE
            );
        }

        prevA = gamepad1.a;
        prevX = gamepad1.x;
        prevY = gamepad1.y;

        shooter.setSpinEnabled(spinEnabled);
        shooter.setAutoAimEnabled(autoAimEnabled);
        shooter.requestFire(gamepad1.right_bumper);
        shooter.update();

        PrecisionShooterSubsystem.TelemetrySnapshot snapshot = shooter.snapshot();
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Alliance", snapshot.alliance);
        telemetry.addData("Homed", snapshot.homed);
        telemetry.addData("Ready", snapshot.ready);
        telemetry.addData("Solution Valid", snapshot.solutionValid);
        telemetry.addData("Target RPM", "%.1f", snapshot.targetRpm);
        telemetry.addData("Actual RPM", "%.1f", snapshot.actualRpm);
        telemetry.addData("Nominal Hood", "%.2f", snapshot.nominalHoodDeg);
        telemetry.addData("Comp Hood", "%.2f", snapshot.compensatedHoodDeg);
        telemetry.addData("Turret", "%.2f / %.2f", snapshot.turretAngleDeg, snapshot.turretTargetDeg);
        telemetry.addData("Predicted Range", "%.2f", snapshot.predictedRangeInches);
        telemetry.addData("TOF", "%.3f", snapshot.timeOfFlightSeconds);
        telemetry.addData("Status", snapshot.status);
        telemetry.addLine("A = spin, RB = fire, X = auto-aim toggle, Y = alliance swap");
        telemetry.update();
    }
}
