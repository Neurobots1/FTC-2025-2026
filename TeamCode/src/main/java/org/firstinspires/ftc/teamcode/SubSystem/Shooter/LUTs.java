package org.firstinspires.ftc.teamcode.SubSystem.Shooter;

import com.seattlesolvers.solverslib.util.InterpLUT;
public class LUTs {

    public static final InterpLUT closeRPM = new InterpLUT();
    public static final InterpLUT midRPM   = new InterpLUT();
    public static final InterpLUT farRPM   = new InterpLUT();

    static {

        // CLOSE
        closeRPM.add(18, 1700);
        closeRPM.add(24, 1780);
        closeRPM.add(30, 1860);
        closeRPM.createLUT();

        // MID
        midRPM.add(32, 1920);
        midRPM.add(38, 2020);
        midRPM.add(44, 2120);
        midRPM.add(50, 2240);
        midRPM.createLUT();

        // FAR
        farRPM.add(52, 2320);
        farRPM.add(58, 2450);
        farRPM.add(64, 2590);
        farRPM.add(70, 2740);
        farRPM.createLUT();
    }

    private LUTs() {}
}
