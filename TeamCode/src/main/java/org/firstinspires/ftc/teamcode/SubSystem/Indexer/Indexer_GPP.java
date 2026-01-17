package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Indexer_GPP {
    private Indexer_Base indexerBase;
    private enum ActionState {IDLE, START, SWAP_TO_LEFT, FINISH}
    public ActionState gppState1 = ActionState.IDLE;
    public ActionState gppState2 = ActionState.IDLE;

    public IntakeMotor intkM;
    public Servo indexLeftServo;
    public Servo indexRightServo;
    public Servo indexGateFront;
    public Servo indexGateBack;
    public static double INDEXER_COLLECT_TIME = 0.5;

    // Timers séparés pour chaque ligne
    private ElapsedTime line1Timer;
    private ElapsedTime line2Timer;

    private final Indexer_Base base;

    public Indexer_GPP(HardwareMap hardwareMap, Indexer_Base base) {
        this.base = base;
        this.intkM          = base.intkM;
        this.indexLeftServo = base.indexLeftServo;
        this.indexRightServo = base.indexRightServo;
        this.indexGateFront = base.indexGateFront;
        this.indexGateBack  = base.indexGateBack;
        this.line1Timer = new ElapsedTime();
        this.line2Timer = new ElapsedTime();
    }

    public boolean isBusy() {
        return (gppState1 == ActionState.START || gppState1 == ActionState.SWAP_TO_LEFT || gppState1 == ActionState.FINISH
                || gppState2 == ActionState.START || gppState2 == ActionState.SWAP_TO_LEFT || gppState2 == ActionState.FINISH);
    }

    public void startLine1() {
        if (isBusy()) return;
        if (gppState1 == ActionState.IDLE || gppState1 == ActionState.FINISH) {
            line1Timer.reset(); // Reset du timer au démarrage
            gppState1 = ActionState.START;
        }
    }

    public void startLine2() {
        if (isBusy()) return;
        if (gppState2 == ActionState.IDLE || gppState2 == ActionState.FINISH) {
            line2Timer.reset(); // Reset du timer au démarrage
            gppState2 = ActionState.START;
        }
    }

    public void Line1() {
        switch (gppState1) {

            case IDLE:
                break;

            case START:
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                intkM.intake();
                line1Timer.reset(); // Reset pour l'état SWAP_TO_LEFT
                gppState1 = ActionState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (line1Timer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                    line1Timer.reset(); // Reset pour l'état FINISH
                    gppState1 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line1Timer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    gppState1 = ActionState.IDLE;
                }
                break;
        }
    }

    public void Line2() {
        switch (gppState2) {

            case IDLE:
                break;

            case START:
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                intkM.intake();
                line2Timer.reset(); // Reset pour l'état SWAP_TO_LEFT
                gppState2 = ActionState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (line2Timer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line2Timer.reset(); // Reset pour l'état FINISH
                    gppState2 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line2Timer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    intkM.stop();
                    gppState2 = ActionState.IDLE;
                }
                break;
        }
    }
}