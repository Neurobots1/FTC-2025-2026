package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import org.firstinspires.ftc.teamcode.SubSystem.Indexer.Indexer_Base;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Indexer_GPP {
    private Indexer_Base indexerBase;
    private enum GPPstate1 {IDLE, START, SWAP_TO_LEFT, FINISH}
    public GPPstate1 gppState1 = GPPstate1.IDLE;

    private enum GPPstate2 {IDLE, START, SWAP_TO_LEFT, FINISH}
    public GPPstate2 gppState2 = GPPstate2.IDLE;



    public IntakeMotor intkM;
    public Servo indexLeftServo;
    public Servo indexRightServo;
    public Servo indexGateFront;
    public Servo indexGateBack;
    public static double INDEXER_COLLECT_TIME = 0.5;
    private ElapsedTime ballEntryTimer;






    public void Line1() {
        switch (gppState1) {

            case IDLE:

                break;

            case START:
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                intkM.intake();
                ballEntryTimer.reset();
                gppState1 = GPPstate1.SWAP_TO_LEFT;
                break;

            case SWAP_TO_LEFT:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                    ballEntryTimer.reset();
                    gppState1 = GPPstate1.FINISH;
                }
                break;


            case FINISH:
                if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                    intkM.stop();
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    gppState1 = GPPstate1.IDLE;

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
                    ballEntryTimer.reset();
                    gppState2 = GPPstate2.SWAP_TO_LEFT;
                    break;

                case SWAP_TO_LEFT:
                    if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                        indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                        indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                        indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                        ballEntryTimer.reset();
                        gppState2 = GPPstate2.FINISH;
                    }
                    break;


                case FINISH:
                    if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
                        indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                        indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                        indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                        intkM.stop();
                        gppState2 = GPPstate2.IDLE;

                    }
                    break;
            }
        }

    }

