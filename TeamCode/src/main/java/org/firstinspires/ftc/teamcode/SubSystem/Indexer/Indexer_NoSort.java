package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;

public class Indexer_NoSort {

    private enum IntakeState {
        IDLE,
        START,
        RUNNING,
        FINISH
    }

    private enum OuttakeState {
        IDLE,
        START,
        RUNNING,
        FINISH
    }

    private IntakeState intakeState = IntakeState.IDLE;
    private OuttakeState outtakeState = OuttakeState.IDLE;

    private final IntakeMotor intkM;

    public Indexer_NoSort(HardwareMap hardwareMap, Indexer_Base base) {
        this.intkM = base.intkM;
    }

    public boolean isBusy() {
        return intakeState != IntakeState.IDLE || outtakeState != OuttakeState.IDLE;
    }

    public void NoSortIntake() {
        switch (intakeState) {

            case IDLE:
                intakeState = IntakeState.START;
                break;

            case START:
                intkM.intake();
                intakeState = IntakeState.RUNNING;
                break;

            case RUNNING:
                break;

            case FINISH:
                intkM.stop();
                intakeState = IntakeState.IDLE;
                break;
        }
    }

    public void NoSortOuttake() {
        switch (outtakeState) {

            case IDLE:
                outtakeState = OuttakeState.START;
                break;

            case START:
                intkM.outtake();
                outtakeState = OuttakeState.RUNNING;
                break;

            case RUNNING:
                break;

            case FINISH:
                intkM.stop();
                outtakeState = OuttakeState.IDLE;
                break;
        }
    }

    public void stopIntake() {
        intkM.stop();
        intakeState = IntakeState.IDLE;
    }

    public void stopOuttake() {
        intkM.stop();
        outtakeState = OuttakeState.IDLE;
    }
}
