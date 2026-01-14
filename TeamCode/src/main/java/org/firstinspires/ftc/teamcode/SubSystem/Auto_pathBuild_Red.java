package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Auto_pathBuild_Red {
    private Follower follower;

    public PathChain TakePatern,Shoot1, Shoot2, Shoot3, Shoot4, IntkSt1, IntkSt2, IntkSt3, IntkFi1, IntkFi2, IntkFi3, FiString;

    public Auto_pathBuild_Red(Follower follower) {
        this.follower = follower;
    }

    private final Pose startPose = new Pose(110, 136, Math.toRadians(90));
    private final Pose SeePatern = new Pose(92, 92,Math.toRadians(90) );
    private final Pose Shoot = new Pose(92, 92, Math.toRadians(50));
    private final Pose IntkStart1 = new Pose(100, 85, Math.toRadians(0));
    private final Pose IntkFinal1 = new Pose(127, 85, Math.toRadians(0));
    private final Pose IntkStart2 = new Pose(100, 63, Math.toRadians(0));
    private final Pose IntkFinal2 = new Pose(127, 63, Math.toRadians(0));
    private final Pose IntkStart3 = new Pose(100, 40, Math.toRadians(0));
    private final Pose IntkFinal3 = new Pose(127, 40, Math.toRadians(0));
    private final Pose FinalShootPose = new Pose(93, 108, Math.toRadians(37));
    public static Pose finalPose = new Pose();

    public void buildPaths() {
        TakePatern = follower.pathBuilder()
                .addPath(new BezierLine(startPose, SeePatern))
                .setLinearHeadingInterpolation(startPose.getHeading(), SeePatern.getHeading())
                .build();

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(SeePatern, Shoot))
                .setLinearHeadingInterpolation(SeePatern.getHeading(), Shoot.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal1, Shoot))
                .setLinearHeadingInterpolation(IntkFinal1.getHeading(), Shoot.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal2, Shoot))
                .setLinearHeadingInterpolation(IntkFinal2.getHeading(), Shoot.getHeading())
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal3, FinalShootPose))
                .setLinearHeadingInterpolation(IntkFinal3.getHeading(), FinalShootPose.getHeading())
                .build();

        IntkSt1 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart1))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart1.getHeading())
                .build();

        IntkFi1 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart1, IntkFinal1))
                .setLinearHeadingInterpolation(IntkStart1.getHeading(), IntkFinal1.getHeading())
                .build();

        IntkSt2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart2))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart2.getHeading())
                .build();

        IntkFi2 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart2, IntkFinal2))
                .setLinearHeadingInterpolation(IntkStart2.getHeading(), IntkFinal2.getHeading())
                .build();

        IntkSt3 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntkStart3))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntkStart3.getHeading())
                .build();

        IntkFi3 = follower.pathBuilder()
                .addPath(new BezierLine(IntkStart3, IntkFinal3))
                .setLinearHeadingInterpolation(IntkStart3.getHeading(), IntkFinal3.getHeading())
                .build();

        FiString = follower.pathBuilder()
                .addPath(new BezierLine(IntkFinal3, FinalShootPose))
                .setLinearHeadingInterpolation(IntkFinal3.getHeading(), FinalShootPose.getHeading())
                .build();
    }

}
