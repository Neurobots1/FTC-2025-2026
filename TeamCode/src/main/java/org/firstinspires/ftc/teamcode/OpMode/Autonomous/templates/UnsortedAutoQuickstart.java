package org.firstinspires.ftc.teamcode.OpMode.Autonomous.templates;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.AutoAlliance;
import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular.ModularAutoBuilder;

public abstract class UnsortedAutoQuickstart extends BaseUnsortedAutoTemplate {

    protected PathChain preloadPath;

    @Override
    protected Pose startPose() {
        // Step 1: define every field pose in BLUE coordinates.
        // Step 2: use paths().pose(...) so red is mirrored automatically.
        return paths().pose(blueStartPose());
    }

    @Override
    protected void buildPaths() {
        // This gives you a ready-to-use preload shot path.
        // Add the rest of your paths inside buildAdditionalPaths().
        preloadPath = paths().shotLine(blueStartPose(), bluePreloadPose());
        buildAdditionalPaths();
    }

    @Override
    protected void buildRoutine() {
        // Add only the action order here. The shared init/update/mirroring is already handled.
        ModularAutoBuilder builder = new ModularAutoBuilder(robot());
        buildActions(builder);
        scheduler().setAction(builder.build());
    }

    // Define the BLUE starting pose only.
    protected abstract Pose blueStartPose();

    // Define the BLUE preload shot pose only.
    protected abstract Pose bluePreloadPose();

    // Create any additional paths here using blue poses and paths().line/curve/shotLine/shotCurve.
    protected abstract void buildAdditionalPaths();

    // Add the auto steps here with the provided builder.
    protected abstract void buildActions(ModularAutoBuilder builder);

    // Use this for the Driver Station blue wrapper.
    public abstract static class Blue extends UnsortedAutoQuickstart {
        @Override
        protected final AutoAlliance alliance() {
            return AutoAlliance.BLUE;
        }
    }

    // Use this for the Driver Station red wrapper.
    public abstract static class Red extends UnsortedAutoQuickstart {
        @Override
        protected final AutoAlliance alliance() {
            return AutoAlliance.RED;
        }
    }
}
