package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("all")
public class Indexer_PGP {
    private Indexer_Base indexerBase;

    private enum ActionState {IDLE, WAIT_FOR_START, START, Color_Detection, SWAP_TO_RIGHT, SWAP_TO_MIDDLE, FINISH, SWAP_TO_LEFT, RESET}

    public ActionState pgpState1 = ActionState.IDLE;
    public ActionState pgpState2 = ActionState.IDLE;
    public ActionState pgpState3 = ActionState.IDLE;
    public ActionState pgpState1_OT = ActionState.IDLE;
    public ActionState pgpState2_OT = ActionState.IDLE;
    public ActionState pgpState3_OT = ActionState.IDLE;

    public IntakeMotor intkM;
    public RevColorSensorV3 colorSensor;
    public Servo indexLeftServo;
    public Servo indexRightServo;
    public Servo indexGateFront;
    public Servo indexGateBack;

    private final LauncherSubsystem Shooter;

    private boolean wantShoot = false;

    private double shootX = 0;
    private double shootY = 0;
    private double shootDistance = 0;

    private ElapsedTime line1IntakeTimer;
    private ElapsedTime line2IntakeTimer;
    private ElapsedTime line3IntakeTimer;
    private ElapsedTime line1OuttakeTimer;
    private ElapsedTime line2OuttakeTimer;
    private ElapsedTime line3OuttakeTimer;

    public Indexer_PGP(HardwareMap hardwareMap, Indexer_Base base, LauncherSubsystem shooter) {
        this.indexerBase = base;
        this.intkM = base.intkM;
        this.indexLeftServo = base.indexLeftServo;
        this.indexRightServo = base.indexRightServo;
        this.indexGateFront = base.indexGateFront;
        this.indexGateBack = base.indexGateBack;

        this.line1IntakeTimer = new ElapsedTime();
        this.line2IntakeTimer = new ElapsedTime();
        this.line3IntakeTimer = new ElapsedTime();
        this.line1OuttakeTimer = new ElapsedTime();
        this.line2OuttakeTimer = new ElapsedTime();
        this.line3OuttakeTimer = new ElapsedTime();

        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        this.Shooter = shooter;
    }

    public boolean isBusy() {
        return (pgpState1 != ActionState.IDLE
                || pgpState2 != ActionState.IDLE
                || pgpState3 != ActionState.IDLE
                || pgpState1_OT != ActionState.IDLE
                || pgpState2_OT != ActionState.IDLE
                || pgpState3_OT != ActionState.IDLE);
    }

    public void setShootContext(double x, double y, double distance) {
        this.shootX = x;
        this.shootY = y;
        this.shootDistance = distance;
    }

    public void startLine1Intake() {
        if (pgpState1 != ActionState.IDLE) return;
        line1IntakeTimer.reset();
        pgpState1 = ActionState.START;
    }

    public void startLine1Outtake() {
        if (pgpState1_OT != ActionState.IDLE) return;
        wantShoot = true;
        line1OuttakeTimer.reset();
        pgpState1_OT = ActionState.RESET;
    }

    public void startLine2Intake() {
        if (pgpState2 != ActionState.IDLE) return;
        line2IntakeTimer.reset();
        pgpState2 = ActionState.START;
    }

    public void startLine2Outtake() {
        if (pgpState2_OT != ActionState.IDLE) return;
        wantShoot = true;
        line2OuttakeTimer.reset();
        pgpState2_OT = ActionState.RESET;
    }

    public void startLine3Intake() {
        if (pgpState3 != ActionState.IDLE) return;
        line3IntakeTimer.reset();
        pgpState3 = ActionState.START;
    }

    public void startLine3Outtake() {
        if (pgpState3_OT != ActionState.IDLE) return;
        wantShoot = true;
        line3OuttakeTimer.reset();
        pgpState3_OT = ActionState.RESET;
    }

    public void Line1Intake() {
        switch (pgpState1) {
            case IDLE:
                break;

            case START:
                intkM.intake();
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                line1IntakeTimer.reset();
                pgpState1 = ActionState.Color_Detection;
                break;

            case Color_Detection:
                if (colorSensor.getDistance(DistanceUnit.MM) <= IndexerTimings.COLOR_DETECT_MM) {
                    line1IntakeTimer.reset();
                    pgpState1 = ActionState.SWAP_TO_MIDDLE;
                }
                break;

            case SWAP_TO_MIDDLE:
                if (line1IntakeTimer.seconds() > IndexerTimings.L1_IN_SWAP_TO_MIDDLE_DELAY_S) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    pgpState1 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line1IntakeTimer.seconds() >= IndexerTimings.L1_IN_FINISH_STOP_S) {
                    intkM.stop();
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
                if (Shooter.flywheelReady()) {
                    intkM.slowIntake();
                    line1OuttakeTimer.reset();
                    pgpState1_OT = ActionState.START;
                }
                break;

            case START:
                if (line1OuttakeTimer.seconds() > IndexerTimings.L1_OUT_START_TO_SWAP_DELAY_S) {
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line1OuttakeTimer.reset();
                    pgpState1_OT = ActionState.SWAP_TO_LEFT;
                }
                break;

            case SWAP_TO_LEFT:
                if (line1OuttakeTimer.seconds() >= IndexerTimings.L1_OUT_SWAP_TO_LEFT_DONE_S) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    wantShoot = false;
                    intkM.stop();
                    pgpState1_OT = ActionState.IDLE;
                }
                break;
        }
    }

    public void Line2Intake() {
        switch (pgpState2) {
            case IDLE:
                break;

            case START:
                intkM.intake();
                indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                line2IntakeTimer.reset();
                pgpState2 = ActionState.Color_Detection;
                break;

            case Color_Detection:
                if (colorSensor.getDistance(DistanceUnit.MM) <= IndexerTimings.COLOR_DETECT_MM) {
                    line2IntakeTimer.reset();
                    pgpState2 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line2IntakeTimer.seconds() >= IndexerTimings.L2_IN_FINISH_STOP_S) {
                    intkM.stop();
                    pgpState2 = ActionState.IDLE;
                }
                break;
        }
    }

    public void Line2Outtake() {
        switch (pgpState2_OT) {
            case IDLE:
                break;

            case RESET:
                indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                if (Shooter.flywheelReady()) {
                    intkM.slowIntake();
                    line2OuttakeTimer.reset();
                    pgpState2_OT = ActionState.START;
                }
                break;

            case START:
                if (line2OuttakeTimer.seconds() > IndexerTimings.L2_OUT_START_DONE_S) {
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    wantShoot = false;
                    intkM.stop();
                    pgpState2_OT = ActionState.IDLE;
                }
                break;
        }
    }

    public void Line3Intake() {
        switch (pgpState3) {
            case IDLE:
                break;

            case START:
                intkM.intake();
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                line3IntakeTimer.reset();
                pgpState3 = ActionState.Color_Detection;
                break;

            case Color_Detection:
                if (colorSensor.getDistance(DistanceUnit.MM) <= IndexerTimings.COLOR_DETECT_MM) {
                    line3IntakeTimer.reset();
                    pgpState3 = ActionState.SWAP_TO_RIGHT;
                }
                break;

            case SWAP_TO_RIGHT:
                if (line3IntakeTimer.seconds() > IndexerTimings.L3_IN_SWAP_TO_RIGHT_DELAY_S) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    line3IntakeTimer.reset();
                    pgpState3 = ActionState.SWAP_TO_LEFT;
                }
                break;

            case SWAP_TO_LEFT:
                if (line3IntakeTimer.seconds() > IndexerTimings.L3_IN_SWAP_TO_LEFT_DELAY_S) {
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line3IntakeTimer.reset();
                    pgpState3 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line3IntakeTimer.seconds() >= IndexerTimings.L3_IN_FINISH_STOP_S) {
                    intkM.stop();
                    pgpState3 = ActionState.IDLE;
                }
                break;
        }
    }

    public void Line3Outtake() {
        switch (pgpState3_OT) {
            case IDLE:
                break;

            case RESET:
                if (Shooter.flywheelReady()) {
                    intkM.slowIntake();
                    line3OuttakeTimer.reset();
                    pgpState3_OT = ActionState.START;
                }
                break;

            case START:
                if (line3OuttakeTimer.seconds() > IndexerTimings.L3_OUT_START_TO_SWAP_DELAY_S) {
                    indexLeftServo.setPosition(Indexer_Base.indexer_R_Engage);
                    indexRightServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line3OuttakeTimer.reset();
                    pgpState3_OT = ActionState.SWAP_TO_LEFT;
                }
                break;

            case SWAP_TO_LEFT:
                if (line3OuttakeTimer.seconds() >= IndexerTimings.L3_OUT_SWAP_TO_LEFT_DELAY_S) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line3OuttakeTimer.reset();
                    pgpState3_OT = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line3OuttakeTimer.seconds() >= IndexerTimings.L3_OUT_FINISH_DONE_S) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    intkM.stop();
                    wantShoot = false;
                    pgpState3_OT = ActionState.IDLE;
                }
                break;
        }
    }

    public void stopAll() {
        pgpState1 = ActionState.IDLE;
        pgpState2 = ActionState.IDLE;
        pgpState3 = ActionState.IDLE;
        pgpState1_OT = ActionState.IDLE;
        pgpState2_OT = ActionState.IDLE;
        pgpState3_OT = ActionState.IDLE;

        wantShoot = false;

        if (intkM != null) intkM.stop();
        if (Shooter != null) Shooter.setFlywheelTicks(0);
        if (indexGateBack != null) indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
    }

    public void update() {
        Shooter.updateShootingAuto(wantShoot, shootX, shootY, shootDistance);
        Shooter.update();

        Line1Intake();
        Line2Intake();
        Line3Intake();

        Line1Outtake();
        Line2Outtake();
        Line3Outtake();

        if (!isBusy() && !wantShoot) {
            Shooter.setFlywheelTicks(0);
        }
    }
}
