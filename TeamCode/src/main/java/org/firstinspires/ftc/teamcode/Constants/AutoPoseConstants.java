package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.Pose;

public final class AutoPoseConstants {

    private AutoPoseConstants() {}

    public static Pose CloseStartPose() {
        return new Pose(20, 119, Math.toRadians(139));
    }

    public static Pose FarStartPose() {
        return new Pose(20, 119, Math.toRadians(139));
    }

    public static Pose CloseShootPose() {
        return new Pose(47, 92, Math.toRadians(139));
    }

    //TODO
    public static Pose FarShootPose() {
        return new Pose(47, 92, Math.toRadians(139));
    }

    public static Pose line1StartPose() {
        return new Pose(50, 88, Math.toRadians(190));
    }

    public static Pose line1FinishPose() {
        return new Pose(23, 82, Math.toRadians(190));
    }

    public static Pose line2StartPose() {
        return new Pose(50, 55, Math.toRadians(180));
    }

    public static Pose line2FinishPoseSorted() {
        return new Pose(15, 55, Math.toRadians(180));
    }

    public static Pose line2FinishPoseUnsorted() {
        return new Pose(13, 55, Math.toRadians(180));
    }

    public static Pose line2ControlPose() {
        return new Pose(45, 55);
    }
    // TODO
    public static Pose line3StartPose() {
        return new Pose(50, 26, Math.toRadians(180));
    }

    // TODO
    public static Pose line3ControlPose() {return new Pose(45, 26);}

    // TODO
    public static Pose line3FinishPose() {
        return new Pose(15, 26, Math.toRadians(180));
    }

    public static Pose gatePose() {
        return new Pose(11, 52, Math.toRadians(144));
    }

    public static Pose gateControlPose() {
        return new Pose(27, 55);
    }

    public static Pose finalShotPose() {
        return new Pose(55, 105, Math.toRadians(145));
    }
}
