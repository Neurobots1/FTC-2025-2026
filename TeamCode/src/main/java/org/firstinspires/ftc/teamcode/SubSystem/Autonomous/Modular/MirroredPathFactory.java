package org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.Constants.Constants;

public class MirroredPathFactory {

    private final Follower follower;
    private final AutoAlliance alliance;
    private final double goalX;
    private final double goalY;
    private final boolean useGoalFacing;

    public MirroredPathFactory(Follower follower, AutoAlliance alliance) {
        this(follower, alliance, 0.0, 0.0, false);
    }

    public MirroredPathFactory(Follower follower,
                               AutoAlliance alliance,
                               double goalX,
                               double goalY,
                               boolean useGoalFacing) {
        this.follower = follower;
        this.alliance = alliance;
        this.goalX = goalX;
        this.goalY = goalY;
        this.useGoalFacing = useGoalFacing;
    }

    public Pose pose(Pose bluePose) {
        return AllianceMirroring.forAlliance(bluePose, alliance);
    }

    public PathChain line(Pose blueStart, Pose blueEnd) {
        Pose start = pose(blueStart);
        Pose end = pose(blueEnd);
        return applyHeading(
                follower.pathBuilder().addPath(new BezierLine(start, end)),
                start,
                end,
                false
        )
                .setBrakingStart(Constants.BRAKING_START)
                .setGlobalDeceleration(Constants.GLOBAL_DECELERATION)
                .build();
    }

    public PathChain curve(Pose blueStart, Pose blueControl, Pose blueEnd) {
        Pose start = pose(blueStart);
        Pose control = pose(blueControl);
        Pose end = pose(blueEnd);
        return applyHeading(
                follower.pathBuilder().addPath(new BezierCurve(start, control, end)),
                start,
                end,
                false
        )
                .setBrakingStart(Constants.BRAKING_START)
                .setGlobalDeceleration(Constants.GLOBAL_DECELERATION)
                .build();
    }

    public PathChain shotLine(Pose blueStart, Pose blueEnd) {
        Pose start = pose(blueStart);
        Pose end = pose(blueEnd);
        return applyHeading(
                follower.pathBuilder().addPath(new BezierLine(start, end)),
                start,
                end,
                true
        )
                .setBrakingStart(Constants.BRAKING_START)
                .setGlobalDeceleration(Constants.GLOBAL_DECELERATION)
                .build();
    }

    public PathChain shotCurve(Pose blueStart, Pose blueControl, Pose blueEnd) {
        Pose start = pose(blueStart);
        Pose control = pose(blueControl);
        Pose end = pose(blueEnd);
        return applyHeading(
                follower.pathBuilder().addPath(new BezierCurve(start, control, end)),
                start,
                end,
                true
        )
                .setBrakingStart(Constants.BRAKING_START)
                .setGlobalDeceleration(Constants.GLOBAL_DECELERATION)
                .build();
    }

    private PathBuilder applyHeading(PathBuilder builder, Pose start, Pose end, boolean faceGoalWhenEnabled) {
        if (faceGoalWhenEnabled && useGoalFacing) {
            return builder.setHeadingInterpolation(HeadingInterpolator.facingPoint(goalX, goalY));
        }
        return builder.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
    }
}
