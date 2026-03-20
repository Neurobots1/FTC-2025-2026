package org.firstinspires.ftc.teamcode.Subsystems.Indexer;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.IndexerConstants;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeMotor;

public class Indexer_Base {

    private enum ActionState { IDLE, START, COLLECTING, DONE }

    private enum OutTakeState {IDLE,OUTAKE, START, SWAP_TO_LEFT, FINISH}
    private enum IndexIntakeState {IDLE, START, SWAP_TO_LEFT, FINISH}

    private ActionState rightPickState = ActionState.IDLE;
    private ActionState leftPickState  = ActionState.IDLE;
    private IndexIntakeState indexIntakeState = IndexIntakeState.IDLE;

    private OutTakeState outTakeState = OutTakeState.IDLE;


    private boolean rightAutoIdle = false;
    private boolean leftAutoIdle  = false;


    public IntakeMotor intkM;
    public Servo indexLeftServo;
    public Servo indexRightServo;
    public Servo indexGateBack;

    public static double INDEXER_MOVE_TIME = 0.5;      // Temps pour deplacer le rail (secondes)
    public static double INDEXER_COLLECT_TIME = 0.5; // Temps de collecte (secondes)
    public static double INDEXER_ROTATE_TIME = 0.2;  // Temps de rotation (secondes)

    private ElapsedTime ballEntryTimer;


    public Indexer_Base(HardwareMap hardwareMap) {
        indexLeftServo = hardwareMap.get(Servo.class, "indexLeftServo");
        indexRightServo = hardwareMap.get(Servo.class, "indexRightServo");
        indexGateBack = hardwareMap.get(Servo.class,"indexGateBack");
        intkM = new IntakeMotor(hardwareMap);
        ballEntryTimer = new ElapsedTime();
    }


    public void StartIndexRightPick() {
        if (isBusy()) return;
        if (rightPickState == ActionState.IDLE || rightPickState == ActionState.DONE) {
            rightPickState = ActionState.START;
        }
    }

    public void IndexBlocker(){
        indexRightServo.setPosition(IndexerConstants.RIGHT_BLOCKER);
    }


    public void StartIndexLeftPick() {
        if (isBusy()) return;
        if (leftPickState == ActionState.IDLE || leftPickState == ActionState.DONE) {
            leftPickState = ActionState.START;
        }
    }

    public void startIndexIntake() {
        if (indexIntakeState == IndexIntakeState.IDLE) {
            indexIntakeState = IndexIntakeState.START;
        }
    }

    public void startOutTake() {
        if (isBusy()) return;
        if (outTakeState == OutTakeState.IDLE) {
            outTakeState = OutTakeState.OUTAKE;
        }
    }



    public void CancelIndexerActions() {
        rightPickState = ActionState.IDLE;
        leftPickState  = ActionState.IDLE;
        indexIntakeState = IndexIntakeState.IDLE;
        outTakeState = OutTakeState.IDLE;
        intkM.stop();
    }


    public void OutTake() {
        switch (outTakeState) {
            case IDLE:
                ballEntryTimer.reset();
                break;

            case OUTAKE:
                intkM.outtake();
                if (ballEntryTimer.seconds()>1) {
                    outTakeState = OutTakeState.START;
                }
                break;

            case START:
                indexGateBack.setPosition(IndexerConstants.BACK_GATE_OPEN);
                indexLeftServo.setPosition(IndexerConstants.LEFT_RETRACTED);
                indexRightServo.setPosition(IndexerConstants.RIGHT_ENGAGED);
                ballEntryTimer.reset();
                outTakeState = OutTakeState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(IndexerConstants.RIGHT_RETRACTED);
                    indexLeftServo.setPosition(IndexerConstants.LEFT_ENGAGED);
                    ballEntryTimer.reset();
                    outTakeState = OutTakeState.FINISH;
                }
                break;

            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(IndexerConstants.LEFT_RETRACTED);
                    outTakeState = OutTakeState.IDLE;
                }
                break;
        }
    }








    public void indexIntake() {
        switch (indexIntakeState) {

            case IDLE:

                break;

            case START:
                indexRightServo.setPosition(IndexerConstants.RIGHT_ENGAGED);
                indexGateBack.setPosition(IndexerConstants.BACK_GATE_CLOSED);
                intkM.intake();
                ballEntryTimer.reset();
                indexIntakeState = IndexIntakeState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(IndexerConstants.RIGHT_RETRACTED);
                    indexLeftServo.setPosition(IndexerConstants.LEFT_ENGAGED);
                    ballEntryTimer.reset();
                    indexIntakeState = IndexIntakeState.FINISH;
                }
                break;



            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(IndexerConstants.LEFT_RETRACTED);
                    indexIntakeState = IndexIntakeState.IDLE;

                }
                break;
        }
    }

    public void Not_active(){
        indexGateBack.setPosition(IndexerConstants.BACK_GATE_OPEN);

        indexRightServo.setPosition(IndexerConstants.RIGHT_RETRACTED);
        indexLeftServo.setPosition(IndexerConstants.LEFT_RETRACTED);
    }

    public void StartIndexPose(){
        indexLeftServo.setPosition(IndexerConstants.LEFT_RETRACTED);
        indexRightServo.setPosition(IndexerConstants.RIGHT_RETRACTED);
        indexGateBack.setPosition(IndexerConstants.BACK_GATE_OPEN);

    }

    public boolean isBusy() {
        boolean rightBusy = (rightPickState == ActionState.START || rightPickState == ActionState.COLLECTING);
        boolean leftBusy  = (leftPickState  == ActionState.START || leftPickState  == ActionState.COLLECTING);
        boolean intakeBusy = (indexIntakeState != IndexIntakeState.IDLE);
        boolean outtakeBusy = (outTakeState != OutTakeState.IDLE);

        return rightBusy || leftBusy || intakeBusy || outtakeBusy;
    }


    public double getTimerSeconds() {
        return ballEntryTimer.seconds();
    }

    public String getLeftState() {
        return leftPickState.toString();
    }

    public String getRightState() {
        return rightPickState.toString();
    }

    public String getIndexIntakeState() {
        return indexIntakeState.toString();
    }







}
