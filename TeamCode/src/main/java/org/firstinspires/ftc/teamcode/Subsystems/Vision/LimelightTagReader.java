package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.HardwareMapConstants;

import java.util.List;

public class LimelightTagReader {

    private final HardwareMap hardwareMap;
    private Limelight3A limelight;

    public LimelightTagReader(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public synchronized void start() {
        if (!CameraConstants.LIMELIGHT_ENABLED) {
            limelight = null;
            return;
        }

        if (limelight == null) {
            try {
                limelight = hardwareMap.get(Limelight3A.class, HardwareMapConstants.LIMELIGHT);
            } catch (Exception ignored) {
                limelight = null;
                return;
            }
        }

        try {
            limelight.pipelineSwitch(CameraConstants.AUTO_TAG_LIMELIGHT_PIPELINE);
            limelight.start();
        } catch (Exception ignored) {
        }
    }

    public synchronized void stop() {
        if (limelight == null) {
            return;
        }

        try {
            limelight.stop();
        } catch (Exception ignored) {
        }
    }

    public synchronized Integer getLatestUsefulTagId() {
        if (!CameraConstants.LIMELIGHT_ENABLED || limelight == null) {
            return null;
        }

        LLResult result;
        try {
            result = limelight.getLatestResult();
        } catch (Exception ignored) {
            return null;
        }

        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials;
        try {
            fiducials = result.getFiducialResults();
        } catch (Exception ignored) {
            return null;
        }

        if (fiducials == null) {
            return null;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            if (id == CameraConstants.SORT_TAG_GPP_ID
                    || id == CameraConstants.SORT_TAG_PGP_ID
                    || id == CameraConstants.SORT_TAG_PPG_ID) {
                return id;
            }
        }

        return null;
    }
}
