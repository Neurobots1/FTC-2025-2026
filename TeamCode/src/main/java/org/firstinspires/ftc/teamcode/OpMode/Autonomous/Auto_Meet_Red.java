package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Actions;

@Autonomous(name = "Auto_Meet_Red", group = "Examples")
public class Auto_Meet_Red extends BaseMeetAuto {

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

    private final Pose startPose = new Pose(110, 136, Math.toRadians(90));
    private final Pose seePatternPose = new Pose(92, 92, Math.toRadians(90));
    private final Pose shootPose = new Pose(92, 92, Math.toRadians(50));
    private final Pose intakeStart1 = new Pose(100, 85, Math.toRadians(0));
    private final Pose intakeFinal1 = new Pose(127, 85, Math.toRadians(0));
    private final Pose intakeStart2 = new Pose(100, 63, Math.toRadians(0));
    private final Pose intakeFinal2 = new Pose(127, 63, Math.toRadians(0));
    private final Pose intakeControl2 = new Pose(100, 65);
    private final Pose intakeStart3 = new Pose(100, 40, Math.toRadians(0));
    private final Pose intakeFinal3 = new Pose(130, 40, Math.toRadians(0));
    private final Pose finalShootPose = new Pose(93, 108, Math.toRadians(37));

    private PathChain takePattern;
    private CyclePaths[] cycles;

    @Override
    protected Pose getStartPose() {
        return startPose;
    }

    @Override
    protected double getGoalX() {
        return 148;
    }

    @Override
    protected double getGoalY() {
        return 140;
    }

    @Override
    protected double getModeSelectTimeoutSeconds() {
        return 2.0;
    }

    @Override
    protected void buildPaths() {
        takePattern = follower.pathBuilder()
                .addPath(new BezierLine(startPose, seePatternPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), seePatternPose.getHeading())
                .build();

        cycles = new CyclePaths[]{
                new CyclePaths(
                        follower.pathBuilder()
                                .addPath(new BezierLine(shootPose, intakeStart1))
                                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStart1.getHeading())
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierLine(intakeStart1, intakeFinal1))
                                .setLinearHeadingInterpolation(intakeStart1.getHeading(), intakeFinal1.getHeading())
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierLine(intakeFinal1, shootPose))
                                .setLinearHeadingInterpolation(intakeFinal1.getHeading(), shootPose.getHeading())
                                .build(),
                        DEFAULT_INTAKE_FINAL_SPEED_L1
                ),
                new CyclePaths(
                        follower.pathBuilder()
                                .addPath(new BezierLine(shootPose, intakeStart2))
                                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStart2.getHeading())
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierLine(intakeStart2, intakeFinal2))
                                .setLinearHeadingInterpolation(intakeStart2.getHeading(), intakeFinal2.getHeading())
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierCurve(intakeFinal2, intakeControl2, shootPose))
                                .setLinearHeadingInterpolation(intakeFinal2.getHeading(), shootPose.getHeading())
                                .build(),
                        DEFAULT_INTAKE_FINAL_SPEED_L2
                ),
                new CyclePaths(
                        follower.pathBuilder()
                                .addPath(new BezierLine(shootPose, intakeStart3))
                                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStart3.getHeading())
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierLine(intakeStart3, intakeFinal3))
                                .setLinearHeadingInterpolation(intakeStart3.getHeading(), intakeFinal3.getHeading())
                                .build(),
                        follower.pathBuilder()
                                .addPath(new BezierLine(intakeFinal3, finalShootPose))
                                .setLinearHeadingInterpolation(intakeFinal3.getHeading(), finalShootPose.getHeading())
                                .build(),
                        DEFAULT_INTAKE_FINAL_SPEED_L3
                )
        };
    }

    @Override
    protected org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction createOpeningAction() {
        return Actions.followPath(follower, takePattern, 1.0, true);
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
        return "Meet Red";
    }
}
