package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Indexer_PGP {
    private Indexer_Base indexerBase;
    private enum PGPstate1 {IDLE, START, SWAP_TO_LEFT, FINISH}
    public PGPstate1 pgpState1 = Indexer_PGP.PGPstate1.IDLE;

    private enum PGPstate3 {IDLE, START, SWAP_TO_LEFT,All_RETRACTED, FINISH}
    public PGPstate3 pgpState3 = Indexer_PGP.PGPstate3.IDLE;


    public IntakeMotor intkM;
    public Servo indexLeftServo;
    public Servo indexRightServo;
    public Servo indexGateFront;
    public Servo indexGateBack;
    public static double INDEXER_COLLECT_TIME = 0.5;
    private ElapsedTime ballEntryTimer;






    public void Line1() {
        switch (pgpState1) {

            case IDLE:

                break;

            case START:
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                intkM.intake();
                ballEntryTimer.reset();
                pgpState1 = Indexer_PGP.PGPstate1.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    ballEntryTimer.reset();
                    pgpState1 = Indexer_PGP.PGPstate1.FINISH;
                }
                break;


            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    pgpState1 = Indexer_PGP.PGPstate1.IDLE;

                }
                break;
        }
    }










    public void Line3() {
        switch (pgpState3) {

            case IDLE:

                break;

            case START:
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                intkM.intake();
                ballEntryTimer.reset();
                pgpState3 = Indexer_PGP.PGPstate3.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                    ballEntryTimer.reset();
                    pgpState3 = PGPstate3.FINISH;
                }
                break;


            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    intkM.slowOuttake();
                    pgpState3 = Indexer_PGP.PGPstate3.IDLE;

                }
                break;
        }
    }
}

