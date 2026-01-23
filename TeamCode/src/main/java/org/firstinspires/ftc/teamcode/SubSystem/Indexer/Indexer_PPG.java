package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("all")
public class Indexer_PPG {
    private Indexer_Base indexerBase;

    private enum ActionState {IDLE, WAIT_FOR_START, START, Color_Detection, SWAP_TO_RIGHT, SWAP_TO_MIDDLE, FINISH, SWAP_TO_LEFT, RESET}

    public ActionState ppgState1 = ActionState.IDLE;
    public ActionState ppgState2 = ActionState.IDLE;
    public ActionState ppgState3 = ActionState.IDLE;
    public ActionState ppgState1_OT = ActionState.IDLE;
    public ActionState ppgState2_OT = ActionState.IDLE;
    public ActionState ppgState3_OT = ActionState.IDLE;

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

    public Indexer_PPG(HardwareMap hardwareMap, Indexer_Base base, LauncherSubsystem shooter) {
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
        return (ppgState1 != ActionState.IDLE
                || ppgState2 != ActionState.IDLE
                || ppgState3 != ActionState.IDLE
                || ppgState1_OT != ActionState.IDLE
                || ppgState2_OT != ActionState.IDLE
                || ppgState3_OT != ActionState.IDLE);
    }

    public void setShootContext(double x, double y, double distance) {
        this.shootX = x;
        this.shootY = y;
        this.shootDistance = distance;
    }

    public void startLine1Intake() {
        if (ppgState1 != ActionState.IDLE) return;
        line1IntakeTimer.reset();
        ppgState1 = ActionState.START;
    }

    public void startLine1Outtake() {
        if (ppgState1_OT != ActionState.IDLE) return;
        wantShoot = true;
        line1OuttakeTimer.reset();
        ppgState1_OT = ActionState.RESET;
    }

    public void startLine2Intake() {
        if (ppgState2 != ActionState.IDLE) return;
        line2IntakeTimer.reset();
        ppgState2 = ActionState.START;
    }

    public void startLine2Outtake() {
        if (ppgState2_OT != ActionState.IDLE) return;
        wantShoot = true;
        line2OuttakeTimer.reset();
        ppgState2_OT = ActionState.RESET;
    }

    public void startLine3Intake() {
        if (ppgState3 != ActionState.IDLE) return;
        line3IntakeTimer.reset();
        ppgState3 = ActionState.START;
    }

    public void startLine3Outtake() {
        if (ppgState3_OT != ActionState.IDLE) return;
        wantShoot = true;
        line3OuttakeTimer.reset();
        ppgState3_OT = ActionState.RESET;
    }

    // ===== REMAP (PPG):
    // PPG Line1 = PGP Line2
    // PPG Line2 = PGP Line3
    // PPG Line3 = PGP Line1

    // ---- Line1 Intake (PGP Line2 Intake code) ----
    public void Line1Intake() {
        switch (ppgState1) {
            case IDLE:
                break;

            case START:
                intkM.intake();
                indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                line1IntakeTimer.reset();
                ppgState1 = ActionState.Color_Detection;
                break;

            case Color_Detection:
                if (colorSensor.getDistance(DistanceUnit.MM) <= 50) {
                    line1IntakeTimer.reset();
                    ppgState1 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line1IntakeTimer.seconds() >= 2) {
                    intkM.stop();
                    ppgState1 = ActionState.IDLE;
                }
                break;
        }
    }

    // ---- Line1 Outtake (PGP Line2 Outtake code) ----
    public void Line1Outtake() {
        switch (ppgState1_OT) {
            case IDLE:
                break;

            case RESET:
                indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                if (Shooter.flywheelReady()) {
                    intkM.slowIntake();
                    line1OuttakeTimer.reset();
                    ppgState1_OT = ActionState.START;
                }
                break;

            case START:
                if (line1OuttakeTimer.seconds() > 3.5) {
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    wantShoot = false;
                    intkM.stop();
                    ppgState1_OT = ActionState.IDLE;
                }
                break;
        }
    }

    // ---- Line2 Intake (PGP Line3 Intake code) ----
    public void Line2Intake() {
        switch (ppgState2) {
            case IDLE:
                break;

            case START:
                intkM.intake();
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                line2IntakeTimer.reset();
                ppgState2 = ActionState.Color_Detection;
                break;

            case Color_Detection:
                if (colorSensor.getDistance(DistanceUnit.MM) <= 50) {
                    line2IntakeTimer.reset();
                    ppgState2 = ActionState.SWAP_TO_RIGHT;
                }
                break;

            case SWAP_TO_RIGHT:
                if (line2IntakeTimer.seconds() > 1) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    line2IntakeTimer.reset();
                    ppgState2 = ActionState.SWAP_TO_LEFT;
                }
                break;

            case SWAP_TO_LEFT:
                if (line2IntakeTimer.seconds() > 1) {
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line2IntakeTimer.reset();
                    ppgState2 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line2IntakeTimer.seconds() >= 2) {
                    intkM.stop();
                    ppgState2 = ActionState.IDLE;
                }
                break;
        }
    }

    // ---- Line2 Outtake (PGP Line3 Outtake code) ----
    public void Line2Outtake() {
        switch (ppgState2_OT) {
            case IDLE:
                break;

            case RESET:
                if (Shooter.flywheelReady()) {
                    intkM.slowIntake();
                    line2OuttakeTimer.reset();
                    ppgState2_OT = ActionState.START;
                }
                break;

            case START:
                if (line2OuttakeTimer.seconds() > 2) {
                    indexLeftServo.setPosition(Indexer_Base.indexer_R_Engage);
                    indexRightServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line2OuttakeTimer.reset();
                    ppgState2_OT = ActionState.SWAP_TO_LEFT;
                }
                break;

            case SWAP_TO_LEFT:
                if (line2OuttakeTimer.seconds() >= 1.5) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line2OuttakeTimer.reset();
                    ppgState2_OT = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line2OuttakeTimer.seconds() >= 1.5) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    intkM.stop();
                    wantShoot = false;
                    ppgState2_OT = ActionState.IDLE;
                }
                break;
        }
    }

    // ---- Line3 Intake (PGP Line1 Intake code) ----
    public void Line3Intake() {
        switch (ppgState3) {
            case IDLE:
                break;

            case START:
                intkM.intake();
                indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
                line3IntakeTimer.reset();
                ppgState3 = ActionState.Color_Detection;
                break;

            case Color_Detection:
                if (colorSensor.getDistance(DistanceUnit.MM) <= 50) {
                    line3IntakeTimer.reset();
                    ppgState3 = ActionState.SWAP_TO_MIDDLE;
                }
                break;

            case SWAP_TO_MIDDLE:
                if (line3IntakeTimer.seconds() > 0.3) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    ppgState3 = ActionState.FINISH;
                }
                break;

            case FINISH:
                if (line3IntakeTimer.seconds() >= 2) {
                    intkM.stop();
                    ppgState3 = ActionState.IDLE;
                }
                break;
        }
    }

    // ---- Line3 Outtake (PGP Line1 Outtake code) ----
    public void Line3Outtake() {
        switch (ppgState3_OT) {
            case IDLE:
                break;

            case RESET:
                if (Shooter.flywheelReady()) {
                    intkM.slowIntake();
                    line3OuttakeTimer.reset();
                    ppgState3_OT = ActionState.START;
                }
                break;

            case START:
                if (line3OuttakeTimer.seconds() > 2) {
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Engage);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    line3OuttakeTimer.reset();
                    ppgState3_OT = ActionState.SWAP_TO_LEFT;
                }
                break;

            case SWAP_TO_LEFT:
                if (line3OuttakeTimer.seconds() >= 1.5) {
                    indexRightServo.setPosition(Indexer_Base.indexer_R_Retracted);
                    indexLeftServo.setPosition(Indexer_Base.indexer_L_Retracted);
                    indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    wantShoot = false;
                    intkM.stop();
                    ppgState3_OT = ActionState.IDLE;
                }
                break;
        }
    }

    public void stopAll() {
        ppgState1 = ActionState.IDLE;
        ppgState2 = ActionState.IDLE;
        ppgState3 = ActionState.IDLE;
        ppgState1_OT = ActionState.IDLE;
        ppgState2_OT = ActionState.IDLE;
        ppgState3_OT = ActionState.IDLE;

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
