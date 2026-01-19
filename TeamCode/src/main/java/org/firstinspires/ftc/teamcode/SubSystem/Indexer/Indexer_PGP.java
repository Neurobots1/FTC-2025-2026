package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Indexer_PGP {
    private Indexer_Base indexerBase;
    private enum ActionState {IDLE, START, SWAP_TO_LEFT, FINISH , RESET}
    public ActionState pgpState1 = ActionState.IDLE;
    public ActionState pgpState3 = ActionState.IDLE;
    public ActionState pgpState1_OT = ActionState.IDLE;

    public IntakeMotor intkM;
    public Servo indexLeftServo;
    public Servo indexRightServo;
    public Servo indexGateFront;
    public Servo indexGateBack;
    public static double INDEXER_COLLECT_TIME = 1;

    private ElapsedTime line1IntakeTimer;
    private ElapsedTime line3IntakeTimer;
    private ElapsedTime line1OuttakeTimer;

    public Indexer_PGP(HardwareMap hardwareMap, Indexer_Base base) {
        this.indexerBase    = base;
        this.intkM          = base.intkM;
        this.indexLeftServo = base.indexLeftServo;
        this.indexRightServo = base.indexRightServo;
        this.indexGateFront = base.indexGateFront;
        this.indexGateBack  = base.indexGateBack;
        this.line1IntakeTimer = new ElapsedTime();
        this.line3IntakeTimer = new ElapsedTime();
        this.line1OuttakeTimer = new ElapsedTime();
    }

    public boolean isBusy() {
        return (pgpState1 != ActionState.IDLE
                || pgpState3 != ActionState.IDLE
                || pgpState1_OT != ActionState.IDLE);
    }

    public void startLine1Intake() {
        if (pgpState1 != ActionState.IDLE) return;
        line1IntakeTimer.reset();
        pgpState1 = ActionState.START;
    }

    public void startLine3Intake() {
        if (pgpState3 != ActionState.IDLE) return;
        line3IntakeTimer.reset();
        pgpState3 = ActionState.START;
    }

    public void startLine1Outtake() {
        if (pgpState1_OT != ActionState.IDLE) return;
        line1OuttakeTimer.reset();
        pgpState1_OT = ActionState.RESET;
    }

    public void Line1Intake() {
        switch (pgpState1) {

            case IDLE:
                break;

            case START:
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                line1IntakeTimer.reset();
                pgpState1 = ActionState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (line1IntakeTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line1IntakeTimer.reset();
                    pgpState1 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line1IntakeTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    pgpState1 = ActionState.IDLE;
                }
                break;
        }
    }

    public void Line1Outtake() {
        switch (pgpState1_OT) {

            case IDLE:
                break;


            case RESET:
                intkM.slowIntake();
                line1IntakeTimer.reset();
                pgpState1_OT = ActionState.START;

                break;

            case START:
                if (line1IntakeTimer.seconds()>2) {
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line1OuttakeTimer.reset();
                    pgpState1_OT = ActionState.SWAP_TO_LEFT;
                }
                break;

            case SWAP_TO_LEFT:
                if (line1OuttakeTimer.seconds() >= 1.5) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    pgpState1_OT = ActionState.IDLE;
                }
                break;
        }
    }

    public void Line3Intake() {
        switch (pgpState3) {

            case IDLE:
                break;

            case START:
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                intkM.intake();
                line3IntakeTimer.reset();
                pgpState3 = ActionState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (line3IntakeTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                    line3IntakeTimer.reset();
                    pgpState3 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line3IntakeTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    intkM.stop();
                    pgpState3 = ActionState.IDLE;
                }
                break;
        }
    }
}