package org.firstinspires.ftc.teamcode.Subsystems.Autonomous.Modular;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Autonomous.AutoAction;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.IndexerMode;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer_PGP;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.PatternMappedIndexer;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.SortPattern;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision.PrecisionShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.LimelightTagReader;

import java.util.EnumMap;

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

    public boolean wantsGoalTrackingControl() {
        return physicalIndexer.wantsGoalTrackingControl();
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

    public AutoAction lockPatternFromAprilTag(LimelightTagReader tagReader, double timeoutSeconds, SortPattern fallbackPattern) {
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

                SortPattern detected = detectPattern(tagReader);
                if (detected != null) {
                    setPattern(detected);
                    finished = true;
                    stopTagCamera(tagReader);
                    return;
                }

                double elapsed = (System.nanoTime() - startNanos) / 1e9;
                if (elapsed >= timeoutSeconds) {
                    setPattern(fallbackPattern);
                    finished = true;
                    stopTagCamera(tagReader);
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

    private static SortPattern detectPattern(LimelightTagReader tagReader) {
        if (tagReader == null) {
            return null;
        }

        Integer tagId;
        try {
            tagId = tagReader.getLatestUsefulTagId();
        } catch (Exception ignored) {
            return null;
        }

        if (tagId == null) {
            return null;
        }

        if (tagId == CameraConstants.SORT_TAG_GPP_ID) return SortPattern.GPP;
        if (tagId == CameraConstants.SORT_TAG_PGP_ID) return SortPattern.PGP;
        if (tagId == CameraConstants.SORT_TAG_PPG_ID) return SortPattern.PPG;
        return null;
    }

    private static void stopTagCamera(LimelightTagReader tagReader) {
        if (tagReader == null) {
            return;
        }
        try {
            tagReader.stop();
        } catch (Exception ignored) {
        }
    }
}
