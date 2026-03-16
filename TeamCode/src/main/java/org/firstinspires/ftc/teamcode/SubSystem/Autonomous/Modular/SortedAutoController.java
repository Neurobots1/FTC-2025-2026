package org.firstinspires.ftc.teamcode.SubSystem.Autonomous.Modular;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystem.Autonomous.AutoAction;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.IndexerMode;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_PGP;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.PatternMappedIndexer;
import org.firstinspires.ftc.teamcode.SubSystem.Indexer.SortPattern;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystem.Vision.AprilTagPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.EnumMap;
import java.util.List;

public class SortedAutoController {

    private final Indexer_PGP physicalIndexer;
    private final EnumMap<SortPattern, IndexerMode> indexersByPattern = new EnumMap<>(SortPattern.class);
    private SortPattern activePattern = SortPattern.PGP;

    public SortedAutoController(HardwareMap hardwareMap, PrecisionShooterSubsystem shooter) {
        Indexer_Base indexerBase = new Indexer_Base(hardwareMap);
        physicalIndexer = new Indexer_PGP(hardwareMap, indexerBase, shooter);
        indexersByPattern.put(SortPattern.PGP, physicalIndexer);
        indexersByPattern.put(SortPattern.GPP, new PatternMappedIndexer(physicalIndexer, SortPattern.GPP));
        indexersByPattern.put(SortPattern.PPG, new PatternMappedIndexer(physicalIndexer, SortPattern.PPG));
        indexersByPattern.put(SortPattern.NOSORT, new PatternMappedIndexer(physicalIndexer, SortPattern.NOSORT));
    }

    public void setPattern(SortPattern pattern) {
        activePattern = pattern;
    }

    public SortPattern getPattern() {
        return activePattern;
    }

    public void setShootContext(double x, double y, double distance) {
        activeIndexer().setShootContext(x, y, distance);
    }

    public void update() {
        activeIndexer().update();
    }

    public boolean isBusy() {
        return activeIndexer().isBusy();
    }

    public boolean isInShootingZone() {
        return activeIndexer().isInShootingZone();
    }

    public boolean isReadyToShoot() {
        return activeIndexer().isReadyToShoot();
    }

    public void setPreSpinEnabled(boolean enabled) {
        activeIndexer().setPreSpinEnabled(enabled);
    }

    public void startIntakeLine(int logicalLine) {
        activeIndexer().startIntake(logicalLine);
    }

    public void startShootLine(int logicalLine) {
        activeIndexer().startOuttake(logicalLine);
    }

    public void startPreloadShot() {
        physicalIndexer.startLine4Outtake();
    }

    public void stopAll() {
        physicalIndexer.stopAll();
    }

    public AutoAction lockPatternFromAprilTag(AprilTagPipeline aprilTag, double timeoutSeconds, SortPattern fallbackPattern) {
        return new AutoAction() {
            private long startNanos;
            private boolean finished;

            @Override
            public void start() {
                startNanos = System.nanoTime();
                finished = false;
            }

            @Override
            public void update() {
                if (finished) {
                    return;
                }

                SortPattern detected = detectPattern(aprilTag);
                if (detected != null) {
                    setPattern(detected);
                    finished = true;
                    stopTagCamera(aprilTag);
                    return;
                }

                double elapsed = (System.nanoTime() - startNanos) / 1e9;
                if (elapsed >= timeoutSeconds) {
                    setPattern(fallbackPattern);
                    finished = true;
                    stopTagCamera(aprilTag);
                }
            }

            @Override
            public boolean isFinished() {
                return finished;
            }
        };
    }

    private IndexerMode activeIndexer() {
        return indexersByPattern.get(activePattern);
    }

    private static SortPattern detectPattern(AprilTagPipeline aprilTag) {
        if (aprilTag == null) {
            return null;
        }

        List<AprilTagDetection> detections;
        try {
            detections = aprilTag.getAllDetections();
        } catch (Exception ignored) {
            return null;
        }

        if (detections == null) {
            return null;
        }

        for (AprilTagDetection detection : detections) {
            if (detection.id == 21) return SortPattern.GPP;
            if (detection.id == 22) return SortPattern.PGP;
            if (detection.id == 23) return SortPattern.PPG;
        }

        return null;
    }

    private static void stopTagCamera(AprilTagPipeline aprilTag) {
        if (aprilTag == null) {
            return;
        }
        try {
            aprilTag.stopCamera();
        } catch (Exception ignored) {
        }
    }
}
