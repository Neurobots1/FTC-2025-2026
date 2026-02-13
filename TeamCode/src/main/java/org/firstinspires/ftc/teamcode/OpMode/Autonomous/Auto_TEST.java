package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class Auto_TEST extends OpMode {


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.000, 120.000),

                                    new Pose(49.000, 92.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(139))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.000, 92.000),

                                    new Pose(43.000, 58.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.000, 58.000),

                                    new Pose(10.000, 58.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.000, 58.000),
                                    new Pose(55.000, 56.000),
                                    new Pose(49.000, 92.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(139))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(49.000, 92.000),
                                    new Pose(39.000, 63.000),
                                    new Pose(11.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(139))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.000, 60.000),
                                    new Pose(39.000, 63.000),
                                    new Pose(49.000, 92.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(139))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(49.000, 92.000),
                                    new Pose(39.000, 63.000),
                                    new Pose(11.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(139))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.000, 60.000),
                                    new Pose(39.000, 63.000),
                                    new Pose(49.000, 92.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(139))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.000, 92.000),

                                    new Pose(39.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(39.000, 84.000),

                                    new Pose(17.000, 84.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.000, 84.000),

                                    new Pose(64.000, 103.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(139))

                    .build();
        }
    }

    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.getPose();

    }

    @Override
    public void init() {
    }
}
