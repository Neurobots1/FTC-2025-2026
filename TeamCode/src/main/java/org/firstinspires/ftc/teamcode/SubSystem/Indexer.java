package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

public class Indexer {
    public IntakeMotor intkM;
    public Servo indexLeftServo;
    public Servo indexRightServo;
    public Servo indexGateFront;
    public Servo indexGateBack;

    public static double indexer_L_Retracted = 1;//Indexer1 Left
    public static double indexer_L_Engage = 0.0;//Indexer1 Center
    public static double indexer_R_Engage = 1.0;//Indexer2 Center
    public static double indexer_R_Retracted = 0.0;//Indexer2 Right

    public static double servointkF_Closed = 1.0;//Servo Intk ferme(les balles ne peuvent pas passer)
    public static double servointkF_Open = 0.0;//Servo Intk ouver(les balles peuvent passer)

    public static double servointkB_Closed = 1.0;//Servo Intk ferme(les balles ne peuvent pas passer)
    public static double servointkB_Open = 0.0;//Servo Intk ouver(les balles peuvent passer)


    public boolean isIndexer_1_AtCenter() {
        return indexLeftServo.getPosition() >= indexer_L_Engage - 0.05
                && indexLeftServo.getPosition() <= indexer_L_Engage + 0.05; // Tolerance de 0.05
    }
    public boolean isIndexer_2_AtCenter() {
        return indexRightServo.getPosition() >= indexer_R_Engage - 0.05
                && indexRightServo.getPosition() <= indexer_R_Retracted + 0.05; // Tolerance de 0.05
    }

    public static double INDEXER_MOVE_TIME = 0.5;      // Temps pour deplacer le rail (secondes)
    public static double INDEXER_COLLECT_TIME = 0.3; // Temps de collecte (secondes)
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
    public void IndexLeft_PickBall() {
        indexGateBack.setPosition(servointkB_Closed);
        indexLeftServo.setPosition(indexer_L_Engage);
        intkM.intake();
        indexGateFront.setPosition(servointkF_Open);


        if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
            indexGateFront.setPosition(servointkF_Closed);
            indexLeftServo.setPosition(indexer_L_Retracted);
        }

    }

    public void IndexRight_PickBall(){
        indexGateBack.setPosition(servointkB_Closed);
        indexRightServo.setPosition(indexer_R_Engage);
        intkM.intake();
        indexGateFront.setPosition(servointkF_Open);


        if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
            indexGateFront.setPosition(servointkF_Closed);
            indexRightServo.setPosition(indexer_R_Retracted);
        }

    }
    public void Indexer_Switch(){
        indexGateBack.setPosition(servointkB_Closed);
        indexLeftServo.setPosition(indexer_L_Engage);
        indexGateFront.setPosition(servointkF_Open);
        intkM.intake();

        if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
            indexGateFront.setPosition(servointkF_Closed);
        }

        indexLeftServo.setPosition(indexer_L_Retracted);
        indexRightServo.setPosition(indexer_R_Engage);
        intkM.intake();
        ballEntryTimer.reset();
        indexGateFront.setPosition(servointkF_Open);

        if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME){
            indexGateFront.setPosition(servointkF_Closed);
        }

    }
    public void Not_active(){
        indexGateBack.setPosition(servointkB_Closed);
        indexGateFront.setPosition(servointkF_Open);

        indexRightServo.setPosition(indexer_R_Retracted);
        indexLeftServo.setPosition(indexer_L_Retracted);
    }

    public void StartIndexPose(){
        indexLeftServo.setPosition(1);
        indexRightServo.setPosition(0);
    }





}
