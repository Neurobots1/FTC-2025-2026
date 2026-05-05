package org.firstinspires.ftc.teamcode.OpMode.Autonomous.Unsorted.balls15;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpMode.Autonomous.templates.BaseUnsortedAutoTemplate;
import org.firstinspires.ftc.teamcode.Subsystems.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoAlliance;

public abstract class Unsorted15BallAuto30 extends BaseUnsortedAutoTemplate {
    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;
    private PathChain path7;
    private PathChain path8;
    private PathChain path9;
    private PathChain path10;
    private PathChain path11;
    private PathChain path12;
    private PathChain[] manualPaths;
    private int nextPathIndex;
    private boolean shooterEnabled;
    private boolean autoShotRequested;
    private boolean previousNextPathButton;
    private boolean previousShooterButton;

    @Override
    protected Pose startPose() {
        return paths().pose(new Pose(34.000, 135.000, Math.toRadians(270)));
    }

    @Override
    protected void buildPaths() {
        Pose startPose = new Pose(34.609, 137.659, Math.toRadians(270));
        Pose firstShotPose = new Pose(54.260, 93.945, Math.toRadians(270));
        Pose line1IntakePose = new Pose(20.993, 85.071);
        Pose shotPose = new Pose(54.260, 93.945);
        Pose line2ControlPose = new Pose(27.000, 53.000);
        Pose line2IntakePose = new Pose(7.305, 56.900);
        Pose line3ControlPose = new Pose(38.000, 60.034);
        Pose line3IntakePose = new Pose(15.359, 64.034, Math.toRadians(180));
        Pose gatePose = new Pose(10.870, 59.579, Math.toRadians(152.58));
        Pose finalPose = new Pose(61.000, 107.000);

        path1 = line(startPose, firstShotPose, false);
        path2 = tangentLine(firstShotPose, line1IntakePose, false);
        path3 = tangentLine(line1IntakePose, shotPose, true);
        path4 = tangentCurve(shotPose, line2ControlPose, line2IntakePose, false);
        path5 = tangentCurve(line2IntakePose, line2ControlPose, shotPose, true);
        path6 = tangentCurve(shotPose, line3ControlPose, line3IntakePose, false);
        path7 = line(line3IntakePose, gatePose, false);
        path8 = tangentLine(gatePose, shotPose, true);
        path9 = tangentCurve(shotPose, line3ControlPose, line3IntakePose, false);
        path10 = line(line3IntakePose, gatePose, false);
        path11 = tangentLine(gatePose, shotPose, true);
        path12 = constantLine(shotPose, finalPose, Math.toRadians(225), false);
        manualPaths = new PathChain[] {
                path1,
                path2,
                path3,
                path4,
                path5,
                path6,
                path7,
                path8,
                path9,
                path10,
                path11,
                path12
        };
    }

    private PathChain line(Pose blueStart, Pose blueEnd, boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading());

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain tangentLine(Pose blueStart, Pose blueEnd, boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierLine(start, end))
                .setTangentHeadingInterpolation();

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain tangentCurve(Pose blueStart, Pose blueControl, Pose blueEnd, boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose control = paths().pose(blueControl);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setTangentHeadingInterpolation();

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    private PathChain constantLine(Pose blueStart, Pose blueEnd, double heading, boolean reversed) {
        Pose start = paths().pose(blueStart);
        Pose end = paths().pose(blueEnd);
        PathBuilder builder = follower().pathBuilder()
                .addPath(new BezierLine(start, end))
                .setConstantHeadingInterpolation(heading);

        if (reversed) {
            builder = builder.setReversed();
        }

        return builder.build();
    }

    @Override
    protected void buildRoutine() {
        nextPathIndex = 0;
        shooterEnabled = false;
        autoShotRequested = false;
        previousNextPathButton = false;
        previousShooterButton = false;
    }

    @Override
    public void loop() {
        robot().updateDriveControl();
        follower().update();
        AutoPoseHandoff.savePose(hardwareMap.appContext, follower().getPose());
        robot().updateSystems();
        updateManualControls();
        scheduler().update();

        telemetry.addData("Alliance", alliance());
        telemetry.addData("Pose", follower().getPose());
        telemetry.addData("FollowerBusy", follower().isBusy());
        telemetry.addData("RoutineIdle", scheduler().isIdle());
        addTelemetry();
        telemetry.update();
    }

    private void updateManualControls() {
        boolean nextPathButton = gamepad1.a;
        boolean shooterButton = gamepad1.y;

        if (nextPathButton && !previousNextPathButton && !follower().isBusy()) {
            followNextPath();
        }

        if (autoShotRequested && !shooterController().isBusy()) {
            autoShotRequested = false;
        }

        if (shooterButton && !previousShooterButton) {
            if (autoShotRequested || shooterController().isBusy()) {
                stopAutoShot();
            } else {
                startAutoShot();
            }
        }

        if (gamepad1.right_bumper) {
            intake().intake();
        } else if (gamepad1.left_bumper) {
            intake().outtake();
        } else if (!shooterEnabled && !shooterController().isBusy()) {
            intake().stop();
        }

        previousNextPathButton = nextPathButton;
        previousShooterButton = shooterButton;
    }

    private void startAutoShot() {
        shooterEnabled = true;
        autoShotRequested = true;
        shooterController().setEnabled(true);
        shooterController().requestAutoFeed(true);
    }

    private void stopAutoShot() {
        shooterEnabled = false;
        autoShotRequested = false;
        shooterController().requestAutoFeed(false);
        shooterController().setEnabled(false);
        intake().stop();
    }

    private void followNextPath() {
        if (manualPaths == null || nextPathIndex >= manualPaths.length) {
            return;
        }

        follower().followPath(manualPaths[nextPathIndex], 1.0, true);
        nextPathIndex++;
    }

    @Override
    protected void addTelemetry() {
        super.addTelemetry();
        int pathCount = manualPaths == null ? 0 : manualPaths.length;
        int displayPath = Math.min(nextPathIndex + 1, pathCount);
        telemetry.addData("Controls", "A next path, RB intake, LB outtake, Y shoot");
        telemetry.addData("NextPath", "%d/%d", displayPath, pathCount);
        telemetry.addData("ShooterEnabled", shooterEnabled);
        telemetry.addData("AutoShotRequested", autoShotRequested);
    }

    public static class Blue extends Unsorted15BallAuto30 {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.BLUE;
        }
    }

    public static class Red extends Unsorted15BallAuto30 {
        @Override
        protected AutoAlliance alliance() {
            return AutoAlliance.RED;
        }
    }
}
