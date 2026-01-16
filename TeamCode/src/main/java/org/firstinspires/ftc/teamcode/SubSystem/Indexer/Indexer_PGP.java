package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Indexer_PGP {
    private Indexer_Base indexerBase;
    private enum ActionState {IDLE, START, SWAP_TO_LEFT, FINISH}
    public ActionState pgpState1 = ActionState.IDLE;
    public ActionState pgpState3 = ActionState.IDLE;


    public IntakeMotor intkM;
    public Servo indexLeftServo;
    public Servo indexRightServo;
    public Servo indexGateFront;
    public Servo indexGateBack;
    public static double INDEXER_COLLECT_TIME = 0.5;
    private ElapsedTime ballEntryTimer;
    public boolean isBusy() {
        boolean isBusy = (pgpState1 == ActionState.START ||pgpState1 == ActionState.SWAP_TO_LEFT || pgpState3 == ActionState.START || pgpState3 == ActionState.SWAP_TO_LEFT);
        return isBusy();
    }

    public void startLine1() {
        if (isBusy()) return;
        if (pgpState1 == ActionState.IDLE || pgpState1 == ActionState.FINISH) {
            pgpState1 = ActionState.START;
        }
    }

    public void startLine3() {
        if (isBusy()) return;
        if (pgpState3 == ActionState.IDLE || pgpState3 == ActionState.FINISH) {
            pgpState3 = ActionState.START;
        }
    }






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
                pgpState1 = ActionState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    ballEntryTimer.reset();
                    pgpState1 = ActionState.FINISH;
                }
                break;


            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    pgpState1 = ActionState.IDLE;

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
                pgpState3 = ActionState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                    ballEntryTimer.reset();
                    pgpState3 = ActionState.FINISH;
                }
                break;


            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    intkM.slowOuttake();
                    pgpState3 = ActionState.IDLE;

                }
                break;
        }
    }
}

