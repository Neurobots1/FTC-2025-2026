package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("all")
public class Indexer_Rapid implements IndexerMode {

    private final Indexer_Base base;

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

    private final ElapsedTime rapidIntakeTimer;
    private final ElapsedTime rapidOuttakeTimer;
    private final ElapsedTime setupOuttakeTimer;

    private enum RapidIntakeState {IDLE, START, COLOR_DETECTION, FINISH}
    private enum RapidOuttakeState {IDLE, START, FINISH}
    private enum SetupOuttakeState {IDLE, START,FINISH}
    private enum FinishOuttakeState {IDLE,START,FINISH}

    private RapidIntakeState intakeState = RapidIntakeState.IDLE;
    private RapidOuttakeState outtakeState = RapidOuttakeState.IDLE;
    private SetupOuttakeState setupOuttakeState = SetupOuttakeState.IDLE;
    private FinishOuttakeState finishOuttakeState = FinishOuttakeState.IDLE;

    public Indexer_Rapid(HardwareMap hardwareMap, Indexer_Base base, LauncherSubsystem shooter) {
        this.base = base;

        this.intkM = base.intkM;
        this.indexLeftServo = base.indexLeftServo;
        this.indexRightServo = base.indexRightServo;
        this.indexGateFront = base.indexGateFront;
        this.indexGateBack = base.indexGateBack;

        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        this.Shooter = shooter;

        this.rapidIntakeTimer = new ElapsedTime();
        this.rapidOuttakeTimer = new ElapsedTime();
        this.setupOuttakeTimer = new ElapsedTime();
    }

    @Override
    public boolean isBusy() {
        return intakeState != RapidIntakeState.IDLE
                || outtakeState != RapidOuttakeState.IDLE
                || setupOuttakeState != SetupOuttakeState.IDLE
                || finishOuttakeState != FinishOuttakeState.IDLE;

    }

    @Override
    public void setShootContext(double x, double y, double distance) {
        this.shootX = x;
        this.shootY = y;
        this.shootDistance = distance;
    }

    public void startRapidIntakeNoSort() {
        if (intakeState != RapidIntakeState.IDLE) return;
        rapidIntakeTimer.reset();
        intakeState = RapidIntakeState.START;
    }

    public void startRapidOuttake() {
        if (outtakeState != RapidOuttakeState.IDLE) return;
        wantShoot = true;
        rapidOuttakeTimer.reset();
        outtakeState = RapidOuttakeState.START;
    }

    public void startSetupOuttake(){
        wantShoot = true;
        if (setupOuttakeState != SetupOuttakeState.IDLE) return;
        setupOuttakeState = SetupOuttakeState.START;
    }

    public void startFinishOuttake(){
        if (finishOuttakeState != FinishOuttakeState.IDLE) return;
        finishOuttakeState = FinishOuttakeState.START;
    }


    private void RapidIntake() {
        switch (intakeState) {
            case IDLE:
                break;

            case START:
                intkM.intake();
                rapidIntakeTimer.reset();
                intakeState = RapidIntakeState.COLOR_DETECTION;
                break;

            case COLOR_DETECTION:
                if (colorSensor.getDistance(DistanceUnit.MM) <= IndexerTimings.COLOR_DETECT_MM) {
                    rapidIntakeTimer.reset();
                    intakeState = RapidIntakeState.FINISH;
                }
                break;

            case FINISH:
                if (rapidIntakeTimer.seconds() >= IndexerTimings.L1_IN_FINISH_STOP_S) {
                    intkM.stop();
                    intakeState = RapidIntakeState.IDLE;
                }
                break;
        }
    }

    private void SetupRapidOuttake() {
        switch (setupOuttakeState) {
            case IDLE:
                break;

            case START:
                if (Shooter.flywheelReady()){
                    setupOuttakeState = SetupOuttakeState.IDLE;
                }
                break;
        }
    }

    public void FinishOuttake(){
        switch (finishOuttakeState){
            case IDLE:
                break;

            case START:
               if (Shooter.flywheelReady()) {
                   intkM.intake();
                   if (setupOuttakeTimer.seconds()>1.7) {
                       finishOuttakeState = FinishOuttakeState.FINISH;
                   }
               }
                break;

            case FINISH:
                wantShoot = false;
                finishOuttakeState = FinishOuttakeState.IDLE;
                break;
        }
    }


    private void RapidOuttake() {
        switch (outtakeState) {
            case IDLE:
                break;

            case START:
                if (Shooter.flywheelReady()&&rapidOuttakeTimer.seconds()>1) {
                    intkM.slowIntake();
                    rapidOuttakeTimer.reset();
                    outtakeState = RapidOuttakeState.FINISH;
                }
                break;

            case FINISH:
                if (rapidOuttakeTimer.seconds() >= 2.0) {
                    wantShoot = false;
                    intkM.stop();
                    outtakeState = RapidOuttakeState.IDLE;
                }
                break;

        }

    }



    @Override
    public void startIntake(int line) {
        startRapidIntakeNoSort();
    }

    @Override
    public void startOuttake(int line) {
        startRapidOuttake();
    }




    @Override
    public void stopAll() {
        intakeState = RapidIntakeState.IDLE;
        outtakeState = RapidOuttakeState.IDLE;

        wantShoot = false;

        if (intkM != null) intkM.stop();
        if (Shooter != null) Shooter.setFlywheelTicks(0);
        if (indexGateBack != null) indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
    }

    @Override
    public void update() {
        Shooter.updateShootingAuto(wantShoot, shootX, shootY, shootDistance);
        Shooter.update();

        RapidIntake();
        RapidOuttake();
        SetupRapidOuttake();
        FinishOuttake();

        if (!isBusy() && !wantShoot) {
            Shooter.setFlywheelTicks(0);
        }
    }

}
