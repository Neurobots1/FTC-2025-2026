package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Indexer {

    private enum ActionState { IDLE, START, COLLECTING, DONE }

    private enum IndexIntakeState {IDLE, START, SWAP_TO_LEFT, CLOSE_FRONT_GATE,WAIT_FOR_GATE, FINISH}

    private ActionState rightPickState = ActionState.IDLE;
    private ActionState leftPickState  = ActionState.IDLE;
    private IndexIntakeState indexIntakeState = IndexIntakeState.IDLE;

    private boolean rightAutoIdle = false;
    private boolean leftAutoIdle  = false;


    public IntakeMotor intkM;
    public Servo indexLeftServo;
    public Servo indexRightServo;
    public Servo indexGateFront;
    public Servo indexGateBack;

    public static double indexer_L_Retracted = 1;//Indexer1 Left
    public static double indexer_L_Engage = 0.0;//Indexer1 Center
    public static double indexer_R_Engage = 1.0;//Indexer2 Center
    public static double indexer_R_Retracted = 0.0;//Indexer2 Right

    public static double servointkF_Closed = 1;//Servo Intk ferme(les balles ne peuvent pas passer)
    public static double servointkF_Open = 0.0;//Servo Intk ouver(les balles peuvent passer)

    public static double servointkB_Closed = 0;//Servo Intk ferme(les balles ne peuvent pas passer)
    public static double servointkB_Open = 1;//Servo Intk ouver(les balles peuvent passer)


    public boolean isIndexer_1_AtCenter() {
        return indexLeftServo.getPosition() >= indexer_L_Engage - 0.05
                && indexLeftServo.getPosition() <= indexer_L_Engage + 0.05; // Tolerance de 0.05
    }
    public boolean isIndexer_2_AtCenter() {
        return indexRightServo.getPosition() >= indexer_R_Engage - 0.05
                && indexRightServo.getPosition() <= indexer_R_Engage + 0.05; // Tolerance de 0.05
    }

    public static double INDEXER_MOVE_TIME = 0.5;      // Temps pour deplacer le rail (secondes)
    public static double INDEXER_COLLECT_TIME = 0.5; // Temps de collecte (secondes)
    public static double INDEXER_ROTATE_TIME = 0.2;  // Temps de rotation (secondes)

    private ElapsedTime ballEntryTimer;


    public Indexer(HardwareMap hardwareMap) {
        indexLeftServo = hardwareMap.get(Servo.class, "indexLeftServo");
        indexRightServo = hardwareMap.get(Servo.class, "indexRightServo");
        indexGateFront = hardwareMap.get(Servo.class,"indexGateFront");
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



    public void CancelIndexerActions() {
        rightPickState = ActionState.IDLE;
        leftPickState  = ActionState.IDLE;
        indexIntakeState = IndexIntakeState.IDLE;
        intkM.stop();
    }


    public void IndexRight_PickBall() {
        switch (rightPickState) {
            case IDLE:
                break;

            case START:
                rightAutoIdle = false;
                indexGateBack.setPosition(servointkB_Closed);
                indexRightServo.setPosition(indexer_R_Engage);
                indexGateFront.setPosition(servointkF_Open);
                indexGateBack.setPosition(servointkB_Closed);
                intkM.intake();
                ballEntryTimer.reset();
                rightPickState = ActionState.COLLECTING;
                break;

            case COLLECTING:
                //intkM.intake();
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexGateFront.setPosition(servointkF_Closed);
                    indexRightServo.setPosition(indexer_R_Retracted);
                    rightAutoIdle = true;
                    rightPickState = ActionState.DONE;
                }

                break;

            case DONE:
                if (rightAutoIdle) {
                    rightAutoIdle = false;
                    intkM.stop();
                    rightPickState = ActionState.IDLE;
                }
                break;

        }
    }





    public void IndexLeft_PickBall() {
        switch (leftPickState) {
            case IDLE:
                break;

            case START:
                leftAutoIdle = false;

                indexGateBack.setPosition(servointkB_Closed);
                indexLeftServo.setPosition(indexer_L_Engage);
                indexGateFront.setPosition(servointkF_Open


                );
                indexGateBack.setPosition(servointkB_Closed);
                intkM.intake();
                ballEntryTimer.reset();
                leftPickState = ActionState.COLLECTING;
                break;

            case COLLECTING:
                //intkM.intake();
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexGateFront.setPosition(servointkF_Closed);
                    indexLeftServo.setPosition(indexer_L_Retracted);
                    leftAutoIdle = true;
                    leftPickState = ActionState.DONE;
                }

                break;

            case DONE:
                if (leftAutoIdle) {
                    leftAutoIdle = false;
                    intkM.stop();
                    leftPickState = ActionState.IDLE;
                }
                break;

        }
    }

    public void indexIntake() {
        switch (indexIntakeState) {

            case IDLE:

                break;

            case START:
                indexRightServo.setPosition(indexer_R_Engage);
                indexGateFront.setPosition(servointkF_Open);
                indexGateBack.setPosition(servointkB_Closed);
                intkM.intake();
                ballEntryTimer.reset();
                indexIntakeState = IndexIntakeState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(indexer_R_Retracted);
                    indexLeftServo.setPosition(indexer_L_Engage);
                    ballEntryTimer.reset();
                    indexIntakeState = IndexIntakeState.CLOSE_FRONT_GATE;
                }
                break;

            case CLOSE_FRONT_GATE:

                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexGateFront.setPosition(servointkF_Closed);
                    ballEntryTimer.reset();
                    indexIntakeState = IndexIntakeState.WAIT_FOR_GATE;
                }
                break;

            case WAIT_FOR_GATE:

                if (ballEntryTimer.seconds() >= 0.5) {
                    intkM.slowIntake();
                    ballEntryTimer.reset();
                    indexIntakeState = IndexIntakeState.FINISH;
                }



                break;

            case FINISH:
                if (ballEntryTimer.seconds() >= 0.3) {
                    intkM.stop();
                    indexLeftServo.setPosition(indexer_L_Retracted);
                    indexIntakeState = IndexIntakeState.IDLE;

                }
                break;
        }
    }

    public void GPP() {
        switch (indexIntakeState) {

            case IDLE:

                break;

            case START:
                indexLeftServo.setPosition(indexer_L_Retracted);
                indexRightServo.setPosition(indexer_R_Engage);
                indexGateBack.setPosition(servointkB_Closed);
                intkM.intake();
                ballEntryTimer.reset();
                indexIntakeState = IndexIntakeState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(indexer_R_Retracted);
                    indexLeftServo.setPosition(indexer_L_Engage);
                    ballEntryTimer.reset();
                    indexIntakeState = IndexIntakeState.FINISH;
                }
                break;


            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(indexer_L_Retracted);
                    indexIntakeState = IndexIntakeState.IDLE;

                }
                break;
        }
    }

    public void PGP() {
        switch (indexIntakeState) {

            case IDLE:

                break;

            case START:
                indexLeftServo.setPosition(indexer_L_Retracted);
                indexRightServo.setPosition(indexer_R_Engage);
                indexGateBack.setPosition(servointkB_Open);
                intkM.intake();
                ballEntryTimer.reset();
                indexIntakeState = IndexIntakeState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(indexer_R_Retracted);
                    indexLeftServo.setPosition(indexer_L_Retracted);
                    ballEntryTimer.reset();
                    indexIntakeState = IndexIntakeState.FINISH;
                }
                break;


            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(indexer_L_Retracted);
                    indexIntakeState = IndexIntakeState.IDLE;

                }
                break;
        }
    }

    public void PPG() {
        switch (indexIntakeState) {

            case IDLE:

                break;

            case START:
                indexLeftServo.setPosition(indexer_L_Retracted);
                indexRightServo.setPosition(indexer_R_Retracted);
                indexGateBack.setPosition(servointkB_Open);
                intkM.intake();
                ballEntryTimer.reset();
                indexIntakeState = IndexIntakeState.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(indexer_R_Engage);
                    indexLeftServo.setPosition(indexer_L_Retracted);
                    indexGateBack.setPosition(servointkB_Closed);
                    ballEntryTimer.reset();
                    indexIntakeState = IndexIntakeState.FINISH;
                }
                break;


            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(indexer_L_Retracted);
                    indexIntakeState = IndexIntakeState.IDLE;

                }
                break;
        }
    }





    public void Not_active(){
        indexGateBack.setPosition(servointkB_Open);
        indexGateFront.setPosition(servointkF_Open);

        indexRightServo.setPosition(indexer_R_Retracted);
        indexLeftServo.setPosition(indexer_L_Retracted);
    }

    public void StartIndexPose(){
        indexLeftServo.setPosition(indexer_L_Retracted);
        indexRightServo.setPosition(indexer_R_Engage);
        indexGateBack.setPosition(servointkB_Open);
        indexGateFront.setPosition(servointkF_Open);

    }

    public boolean isBusy() {
        boolean rightBusy = (rightPickState == ActionState.START || rightPickState == ActionState.COLLECTING);
        boolean leftBusy  = (leftPickState  == ActionState.START || leftPickState  == ActionState.COLLECTING);
        boolean intakeBusy = (indexIntakeState != IndexIntakeState.IDLE);

        return rightBusy || leftBusy || intakeBusy;
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
