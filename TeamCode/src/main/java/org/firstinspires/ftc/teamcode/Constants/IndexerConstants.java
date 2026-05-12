package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class IndexerConstants {

    public static double LEFT_RETRACTED = 1.0;
    public static double LEFT_ENGAGED = 0.0;

    public static double RIGHT_ENGAGED = 1.0;
    public static double RIGHT_BLOCKER = 0.5;
    public static double RIGHT_RETRACTED = 0.0;

    public static double BACK_GATE_CLOSED = 0.6;
    public static double BACK_GATE_OPEN = 0.3;

    private IndexerConstants() {
    }
}
