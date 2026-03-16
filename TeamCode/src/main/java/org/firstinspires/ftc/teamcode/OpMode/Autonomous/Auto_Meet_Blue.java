package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Actions;

@Autonomous
public class Auto_Meet_Blue extends BaseMeetAuto {

    public static double DEFAULT_INTAKE_FINAL_SPEED_L1 = 1;
    public static double DEFAULT_INTAKE_FINAL_SPEED_L2 = 1;
    public static double DEFAULT_INTAKE_FINAL_SPEED_L3 = 1;
    public static double PGP_INTAKE_FINAL_SPEED_L1 = 1;
    public static double PGP_INTAKE_FINAL_SPEED_L2 = 1;
    public static double PGP_INTAKE_FINAL_SPEED_L3 = 1;
    public static double PPG_INTAKE_FINAL_SPEED_L1 = 1;
    public static double PPG_INTAKE_FINAL_SPEED_L2 = 1;
    public static double PPG_INTAKE_FINAL_SPEED_L3 = 1;
    public static double GPP_INTAKE_FINAL_SPEED_L1 = 1;
    public static double GPP_INTAKE_FINAL_SPEED_L2 = 1;
    public static double GPP_INTAKE_FINAL_SPEED_L3 = 1;
    public static double NOSORT_INTAKE_FINAL_SPEED_L1 = 1;
    public static double NOSORT_INTAKE_FINAL_SPEED_L2 = 1;
    public static double NOSORT_INTAKE_FINAL_SPEED_L3 = 1;

    private final Pose startPose = new Pose(20, 119, Math.toRadians(139));
    private final Pose shootPose = new Pose(47, 92, Math.toRadians(139));
    private final Pose intakeStart1 = new Pose(50, 88, Math.toRadians(190));
    private final Pose intakeFinal1 = new Pose(23, 82, Math.toRadians(190));
    private final Pose intakeStart2 = new Pose(50, 55, Math.toRadians(180));
    private final Pose intakeFinal2 = new Pose(15, 55, Math.toRadians(180));
    private final Pose intakeControl2 = new Pose(45, 55);
    private final Pose intakeStart3 = new Pose(50, 38, Math.toRadians(180));
    private final Pose intakeFinal3 = new Pose(8, 38, Math.toRadians(180));
    private final Pose finalShootPose = new Pose(55, 105, Math.toRadians(145));
    private final Pose gateOpenPose = new Pose(15, 73, Math.toRadians(90));

    private PathChain openGate;
    private PathChain shootPre;
    private CyclePaths[] cycles;

    @Override
    protected Pose getStartPose() {
        return startPose;
    }

    @Override
    protected double getGoalX() {
        return 0;
    }

    @Override
    protected double getGoalY() {
        return 140;
    }

    @Override
    protected double getModeSelectTimeoutSeconds() {
        return 1.0;
    }

    @Override
    protected void buildPaths() {
        openGate = follower.pathBuilder()
                .addPath(new BezierLine(intakeFinal1, gateOpenPose))
                .setLinearHeadingInterpolation(intakeFinal1.getHeading(), gateOpenPose.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        shootPre = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setBrakingStart(0.2)
                .setGlobalDeceleration(0.50)
                .build();

        cycles = new CyclePaths[]{
                new CyclePaths(
                        follower.pathBuilder()
                                .addPath(new BezierLine(shootPose, intakeStart1))
                                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStart1.getHeading())
                                .setBrakingStart(0.2)
                                .setGlobalDeceleration(0.50)
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierLine(intakeStart1, intakeFinal1))
                                .setLinearHeadingInterpolation(intakeStart1.getHeading(), intakeFinal1.getHeading())
                                .setBrakingStart(0.2)
                                .setGlobalDeceleration(0.50)
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierLine(intakeFinal1, shootPose))
                                .setLinearHeadingInterpolation(intakeFinal1.getHeading(), shootPose.getHeading())
                                .setBrakingStart(0.2)
                                .setGlobalDeceleration(0.50)
                                .build(),
                        DEFAULT_INTAKE_FINAL_SPEED_L1
                ),
                new CyclePaths(
                        follower.pathBuilder()
                                .addPath(new BezierLine(shootPose, intakeStart2))
                                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStart2.getHeading())
                                .setBrakingStart(0.2)
                                .setGlobalDeceleration(0.50)
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierLine(intakeStart2, intakeFinal2))
                                .setLinearHeadingInterpolation(intakeStart2.getHeading(), intakeFinal2.getHeading())
                                .setBrakingStart(0.2)
                                .setGlobalDeceleration(0.50)
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierCurve(intakeFinal2, intakeControl2, shootPose))
                                .setLinearHeadingInterpolation(intakeFinal2.getHeading(), shootPose.getHeading())
                                .setBrakingStart(0.2)
                                .setGlobalDeceleration(0.50)
                                .build(),
                        DEFAULT_INTAKE_FINAL_SPEED_L2
                ),
                new CyclePaths(
                        follower.pathBuilder()
                                .addPath(new BezierLine(shootPose, intakeStart3))
                                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStart3.getHeading())
                                .setBrakingStart(0.2)
                                .setGlobalDeceleration(0.50)
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierLine(intakeStart3, intakeFinal3))
                                .setLinearHeadingInterpolation(intakeStart3.getHeading(), intakeFinal3.getHeading())
                                .setBrakingStart(0.2)
                                .setGlobalDeceleration(0.50)
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierLine(intakeFinal3, finalShootPose))
                                .setLinearHeadingInterpolation(intakeFinal3.getHeading(), finalShootPose.getHeading())
                                .setBrakingStart(0.2)
                                .setGlobalDeceleration(0.50)
                                .build(),
                        DEFAULT_INTAKE_FINAL_SPEED_L3
                )
        };
    }

    @Override
    protected org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction createOpeningAction() {
        return Actions.sequence(
                Actions.instant(() -> indexerPgp.setPreSpinEnabled(true)),
                Actions.followPath(follower, shootPre, 1.0, true),
                Actions.instant(() -> indexerPgp.startLine4Outtake()),
                Actions.waitUntil(() -> !indexerPgp.isBusy()),
                Actions.followPath(follower, openGate, 1.0, true)
        );
    }

    @Override
    protected CyclePaths[] getCyclePaths() {
        return cycles;
    }

    @Override
    protected double getIntakeFinishSpeed(int line) {
        if (usePgpMode) {
            return line == 1 ? PGP_INTAKE_FINAL_SPEED_L1 : line == 2 ? PGP_INTAKE_FINAL_SPEED_L2 : PGP_INTAKE_FINAL_SPEED_L3;
        }
        if (usePpgMode) {
            return line == 1 ? PPG_INTAKE_FINAL_SPEED_L1 : line == 2 ? PPG_INTAKE_FINAL_SPEED_L2 : PPG_INTAKE_FINAL_SPEED_L3;
        }
        if (useGppMode) {
            return line == 1 ? GPP_INTAKE_FINAL_SPEED_L1 : line == 2 ? GPP_INTAKE_FINAL_SPEED_L2 : GPP_INTAKE_FINAL_SPEED_L3;
        }
        if (useNoSortMode) {
            return line == 1 ? NOSORT_INTAKE_FINAL_SPEED_L1 : line == 2 ? NOSORT_INTAKE_FINAL_SPEED_L2 : NOSORT_INTAKE_FINAL_SPEED_L3;
        }
        return line == 1 ? DEFAULT_INTAKE_FINAL_SPEED_L1 : line == 2 ? DEFAULT_INTAKE_FINAL_SPEED_L2 : DEFAULT_INTAKE_FINAL_SPEED_L3;
    }

    @Override
    protected String getAutoLabel() {
        return "Meet Blue";
    }
}
