package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IndexerTimings {


    public static double L1_IN_SWAP_TO_MIDDLE_DELAY_S = 0.5;
    public static double L1_IN_FINISH_STOP_S = 2.0;

    public static double L1_OUT_START_TO_SWAP_DELAY_S = 1;
    public static double L1_OUT_SWAP_TO_LEFT_DONE_S = 1;

    public static double L2_IN_FINISH_STOP_S = 2.0;
    public static double L2_OUT_START_DONE_S = 2.5;

    public static double L3_IN_SWAP_TO_RIGHT_DELAY_S = 0.2;
    public static double L3_IN_SWAP_TO_LEFT_DELAY_S = 0.7;
    public static double L3_IN_FINISH_STOP_S = 1.5;

    public static double L3_OUT_START_TO_SWAP_DELAY_S = 0.5;
    public static double L3_OUT_SWAP_TO_LEFT_DELAY_S = 0.7;
    public static double L3_OUT_FINISH_DONE_S = 1.0;


    public static double COLOR_DETECT_MM = 50.0;

    private IndexerTimings() {}
}

