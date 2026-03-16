package org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Constants.Constants;

public class MirroredPathFactory {

    private final Follower follower;
    private final AutoAlliance alliance;

    public MirroredPathFactory(Follower follower, AutoAlliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
    }

    public Pose pose(Pose bluePose) {
        return AllianceMirroring.forAlliance(bluePose, alliance);
    }

    public PathChain line(Pose blueStart, Pose blueEnd) {
        Pose start = pose(blueStart);
        Pose end = pose(blueEnd);
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .setBrakingStart(Constants.BRAKING_START)
                .setGlobalDeceleration(Constants.GLOBAL_DECELERATION)
                .build();
    }

    public PathChain curve(Pose blueStart, Pose blueControl, Pose blueEnd) {
        Pose start = pose(blueStart);
        Pose control = pose(blueControl);
        Pose end = pose(blueEnd);
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .setBrakingStart(Constants.BRAKING_START)
                .setGlobalDeceleration(Constants.GLOBAL_DECELERATION)
                .build();
    }
}
