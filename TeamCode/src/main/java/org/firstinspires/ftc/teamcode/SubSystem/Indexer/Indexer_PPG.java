package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

public class Indexer_PPG {
    private Indexer_Base indexerBase;
    private enum ActionState {IDLE, START, SWAP_TO_LEFT, FINISH}
    public ActionState ppgState2 = ActionState.IDLE;
    public ActionState ppgState3 = ActionState.IDLE;



    public IntakeMotor intkM;
    public Servo indexLeftServo;
    public Servo indexRightServo;
    public Servo indexGateFront;
    public Servo indexGateBack;
    public static double INDEXER_COLLECT_TIME = 0.5;
    private ElapsedTime ballEntryTimer;

    public boolean isBusy() {
        boolean isBusy = (ppgState2 == ActionState.START ||ppgState2 == ActionState.SWAP_TO_LEFT || ppgState3 == ActionState.START || ppgState3 == ActionState.SWAP_TO_LEFT);
        return isBusy();
    }

    public void startLine2() {
        if (isBusy()) return;
        if (ppgState2 == ActionState.IDLE || ppgState2 == ActionState.FINISH) {
            ppgState2 = ActionState.START;
        }
    }

    public void startLine3() {
        if (isBusy()) return;
        if (ppgState3 == ActionState.IDLE || ppgState3 == ActionState.FINISH) {
            ppgState3 = ActionState.START;
        }
    }

    public void Line2() {
        switch (ppgState2) {

            case IDLE:

                break;

            case START:
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                intkM.intake();
                ballEntryTimer.reset();
                ppgState2 = ActionState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                    ballEntryTimer.reset();
                    ppgState2 = ActionState.FINISH;
                }
                break;


            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    ppgState2 = ActionState.IDLE;

                }
                break;
        }
    }






    public void Line3() {
        switch (ppgState3) {

            case IDLE:

                break;

            case START:
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                intkM.intake();
                ballEntryTimer.reset();
                ppgState3 = ActionState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    ballEntryTimer.reset();
                    ppgState3 = ActionState.FINISH;
                }
                break;


            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    intkM.stop();
                    ppgState3 = ActionState.IDLE;

                }
                break;
        }
    }

}

