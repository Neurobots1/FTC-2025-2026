package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.geometry.Pose;

public final class AllianceSelector {

    public enum Alliance { RED, BLUE }

    public interface Provider {
        Alliance getAlliance();

        default void setAlliance(Alliance a) {}  // <-- FIXED: now default

        default void toggle() {
            setAlliance(getAlliance() == Alliance.BLUE ? Alliance.RED : Alliance.BLUE);
        }
    }


    public static final class Manager implements Provider {
        public Alliance alliance;
        public Manager(Alliance start) { this.alliance = start; }
        @Override public Alliance getAlliance() { return alliance; }
        @Override public void setAlliance(Alliance a) { this.alliance = a; }
    }

    public static final class Field {

        public static final double RED_GOAL_X_IN  = 132;
        public static final double RED_GOAL_Y_IN  = 132;
        public static final double BLUE_GOAL_X_IN = 12;
        public static final double BLUE_GOAL_Y_IN = 132;

        public static double goalX(Alliance a) {
            return a == Alliance.BLUE ? BLUE_GOAL_X_IN : RED_GOAL_X_IN;
        }

        public static double goalY(Alliance a) {
            return a == Alliance.BLUE ? BLUE_GOAL_Y_IN : RED_GOAL_Y_IN;
        }

        public static double headingToGoal(Pose pose, Alliance alliance) {
            double gx = goalX(alliance);
            double gy = goalY(alliance);
            double dx = gx - pose.getX();
            double dy = gy - pose.getY();
            return Math.atan2(dy, dx);
        }
    }

    public static final Provider defaultProvider = new Provider() {
        private Alliance alliance = Alliance.RED;

        @Override
        public Alliance getAlliance() {
            return alliance;
        }

        @Override
        public void setAlliance(Alliance a) {
            alliance = a;
        }
    };


    private AllianceSelector() {}
}

