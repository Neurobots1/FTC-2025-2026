package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static double INDEXER_COLLECT_TIME = 1;

    // Timers séparés pour chaque ligne
    private ElapsedTime line2Timer;
    private ElapsedTime line3Timer;

    public Indexer_PPG(HardwareMap hardwareMap, Indexer_Base base) {
        this.indexerBase    = base;
        this.intkM          = base.intkM;
        this.indexLeftServo = base.indexLeftServo;
        this.indexRightServo = base.indexRightServo;
        this.indexGateFront = base.indexGateFront;
        this.indexGateBack  = base.indexGateBack;
        this.line2Timer = new ElapsedTime();
        this.line3Timer = new ElapsedTime();
    }

    public boolean isBusy() {
        return (ppgState2 == ActionState.START || ppgState2 == ActionState.SWAP_TO_LEFT || ppgState2 == ActionState.FINISH
                || ppgState3 == ActionState.START || ppgState3 == ActionState.SWAP_TO_LEFT || ppgState3 == ActionState.FINISH);
    }

    public void startLine2() {
        if (isBusy()) return;
        if (ppgState2 == ActionState.IDLE || ppgState2 == ActionState.FINISH) {
            line2Timer.reset(); // Reset du timer au démarrage
            ppgState2 = ActionState.START;
        }
    }

    public void startLine3() {
        if (isBusy()) return;
        if (ppgState3 == ActionState.IDLE || ppgState3 == ActionState.FINISH) {
            line3Timer.reset(); // Reset du timer au démarrage
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
                line2Timer.reset(); // Reset pour l'état SWAP_TO_LEFT
                ppgState2 = ActionState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (line2Timer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line2Timer.reset(); // Reset pour l'état FINISH
                    ppgState2 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line2Timer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                    intkM.stop();
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
                line3Timer.reset(); // Reset pour l'état SWAP_TO_LEFT
                ppgState3 = ActionState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (line3Timer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                    line3Timer.reset(); // Reset pour l'état FINISH
                    ppgState3 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line3Timer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    intkM.slowOuttake();
                    ppgState3 = ActionState.IDLE;
                }
                break;
        }
    }
}