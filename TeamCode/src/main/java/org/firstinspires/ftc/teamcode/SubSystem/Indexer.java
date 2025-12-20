package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

public class Indexer {
    public IntakeMotor intkM;

    public Servo servoindexer1;
    public Servo servoindexer2;
    public Servo servoIntk;

    public static double indexer1_Left = 0.0;//Indexer1 Left
    public static double indexer1_Center = 0.5;//Indexer1 Center

    public static double indexer2_Center = 0.5;//Indexer2 Center
    public static double indexer2_Right = 1.0;//Indexer2 Right

    public static double servointk_Closed = 1.0;//Servo Intk ferme(les balles ne peuvent pas passer)
    public static double servointk_Open = 0.0;//Servo Intk ouver(les balles peuvent passer)

    public boolean isIndexer_1_AtCenter() {
        return servoindexer1.getPosition() >= indexer1_Center - 0.05
                && servoindexer1.getPosition() <= indexer1_Center + 0.05; // Tolerance de 0.05
    }
    public boolean isIndexer_2_AtCenter() {
        return servoindexer2.getPosition() >= indexer1_Center - 0.05
                && servoindexer2.getPosition() <= indexer1_Center + 0.05; // Tolerance de 0.05
    }

    public static double INDEXER_MOVE_TIME = 0.5;      // Temps pour deplacer le rail (secondes)
    public static double INDEXER_COLLECT_TIME = 0.3; // Temps de collecte (secondes)
    public static double INDEXER_ROTATE_TIME = 0.2;  // Temps de rotation (secondes)

    private ElapsedTime ballEntryTimer;


    public Indexer(HardwareMap hardwareMap) {
        servoindexer1 = hardwareMap.get(Servo.class, "servoindexer1");
        servoindexer2 = hardwareMap.get(Servo.class, "servoindexer2");
        ballEntryTimer = new ElapsedTime();
    }
    public void Caroussel1PickBall() {
        servoindexer1.setPosition(indexer1_Center);
        intkM.intake();
        servoIntk.setPosition(servointk_Open);


        if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
            servoIntk.setPosition(servointk_Closed);
            servoindexer1.setPosition(indexer1_Left);
        }

    }

    public void Caroussel_2_PickBall(){
        servoindexer1.setPosition(indexer2_Center);
        intkM.intake();
        servoIntk.setPosition(servointk_Open);


        if (ballEntryTimer.seconds() >= INDEXER_COLLECT_TIME) {
            servoIntk.setPosition(servointk_Closed);
            servoindexer2.setPosition(indexer2_Right);
        }

    }

    public void Not_active(){
        servoindexer1.setPosition(indexer2_Right);
        servoindexer2.setPosition(indexer1_Left);
    }





}
