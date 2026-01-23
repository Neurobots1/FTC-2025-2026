package org.firstinspires.ftc.teamcode.SubSystem.Indexer;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystem.IntakeMotor;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.LauncherSubsystem;

@SuppressWarnings("all")
public class Indexer_NoSort {

    private enum IntakeState { IDLE, START, Color_Detection, FINISH }
    private enum OuttakeState { IDLE, RESET, START }

    public IntakeState intakeState = IntakeState.IDLE;
    public OuttakeState outtakeState = OuttakeState.IDLE;

    private final IntakeMotor intkM;
    private final Indexer_Base base;
    private final LauncherSubsystem Shooter;

    private final RevColorSensorV3 colorSensor;

    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime outtakeTimer = new ElapsedTime();

    private boolean wantShoot = false;

    private double shootX = 0;
    private double shootY = 0;
    private double shootDistance = 0;

    public Indexer_NoSort(HardwareMap hardwareMap, Indexer_Base base, LauncherSubsystem shooter) {
        this.base = base;
        this.intkM = base.intkM;
        this.Shooter = shooter;
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    }

    public boolean isBusy() {
        return intakeState != IntakeState.IDLE || outtakeState != OuttakeState.IDLE;
    }

    public void setShootContext(double x, double y, double distance) {
        this.shootX = x;
        this.shootY = y;
        this.shootDistance = distance;
    }

    public void startNoSortIntake() {
        if (intakeState != IntakeState.IDLE) return;
        intakeTimer.reset();
        intakeState = IntakeState.START;
    }

    public void startNoSortOuttake() {
        if (outtakeState != OuttakeState.IDLE) return;
        wantShoot = true;
        outtakeTimer.reset();
        outtakeState = OuttakeState.RESET;
    }

    // EXACT STRUCTURE OF PGP Line2Intake:
    // START -> Color_Detection (wait distance <= 50) -> FINISH (2s) -> IDLE
    public void NoSortIntake() {
        switch (intakeState) {
            case IDLE:
                break;

            case START:
                intkM.intake();
                if (base.indexGateBack != null) {
                    base.indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                }
                intakeTimer.reset();
                intakeState = IntakeState.Color_Detection;
                break;

            case Color_Detection:
                if (colorSensor.getDistance(DistanceUnit.MM) <= IndexerTimings.COLOR_DETECT_MM) {
                    intakeTimer.reset();
                    intakeState = IntakeState.FINISH;
                }
                break;

            case FINISH:
                if (intakeTimer.seconds() >= IndexerTimings.L2_IN_FINISH_STOP_S) {
                    intkM.stop();
                    intakeState = IntakeState.IDLE;
                }
                break;
        }
    }

    // EXACT STRUCTURE OF PGP Line2Outtake:
    // RESET (set gate open, wait flywheelReady) -> START (3.5s) -> IDLE + wantShoot false + stop intake
    public void NoSortOuttake() {
        switch (outtakeState) {
            case IDLE:
                break;

            case RESET:
                if (base.indexGateBack != null) {
                    base.indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                }
                if (Shooter.flywheelReady()) {
                    intkM.slowIntake();
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.START;
                }
                break;

            case START:
                if (outtakeTimer.seconds() > IndexerTimings.L2_OUT_START_DONE_S) {
                    if (base.indexGateBack != null) {
                        base.indexGateBack.setPosition(Indexer_Base.servointkB_Open);
                    }
                    wantShoot = false;
                    intkM.stop();
                    outtakeState = OuttakeState.IDLE;
                }
                break;
        }
    }

    public void stopAll() {
        intakeState = IntakeState.IDLE;
        outtakeState = OuttakeState.IDLE;

        wantShoot = false;

        if (intkM != null) intkM.stop();
        if (Shooter != null) Shooter.setFlywheelTicks(0);

        if (base.indexGateBack != null) {
            base.indexGateBack.setPosition(Indexer_Base.servointkB_Closed);
        }
    }

    public void update() {
        Shooter.updateShootingAuto(wantShoot, shootX, shootY, shootDistance);
        Shooter.update();

        NoSortIntake();
        NoSortOuttake();

        if (!isBusy() && !wantShoot) {
            Shooter.setFlywheelTicks(0);
        }
    }
}
