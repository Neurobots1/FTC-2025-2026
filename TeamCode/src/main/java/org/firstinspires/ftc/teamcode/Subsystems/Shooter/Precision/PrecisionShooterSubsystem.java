package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Precision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.LUTConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterHardwareConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.TurretHomeHandoff;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShootingZones;

public final class PrecisionShooterSubsystem {
    private static final double CHASSIS_AIM_COMMAND_LIMIT = 1.0;
    private static final double CHASSIS_AIM_SETTLED_ERROR_RADIANS = Math.toRadians(0.15);
    private static final double CHASSIS_AIM_SETTLED_OMEGA_RADIANS = Math.toRadians(3.0);
    private static final double CHASSIS_AIM_MAX_INTEGRAL_TERM = 0.25;

    public enum Alliance {
        BLUE,
        RED
    }

    public static final class TelemetrySnapshot {
        public final boolean homed;
        public final boolean solutionValid;
        public final boolean inShootingZone;
        public final boolean ready;
        public final double targetRpm;
        public final double actualRpm;
        public final double compensationRpm;
        public final double nominalHoodDeg;
        public final double compensatedHoodDeg;
        public final double aimGoalX;
        public final double aimGoalY;
        public final double goalForwardOffsetInches;
        public final double turretAngleDeg;
        public final double turretTargetDeg;
        public final double predictedRangeInches;
        public final double tableDistanceInches;
        public final double timeOfFlightSeconds;
        public final String status;
        public final Alliance alliance;

        TelemetrySnapshot(boolean homed,
                          boolean solutionValid,
                          boolean inShootingZone,
                          boolean ready,
                          double targetRpm,
                          double actualRpm,
                          double compensationRpm,
                          double nominalHoodDeg,
                          double compensatedHoodDeg,
                          double aimGoalX,
                          double aimGoalY,
                          double goalForwardOffsetInches,
                          double turretAngleDeg,
                          double turretTargetDeg,
                          double predictedRangeInches,
                          double tableDistanceInches,
                          double timeOfFlightSeconds,
                          String status,
                          Alliance alliance) {
            this.homed = homed;
            this.solutionValid = solutionValid;
            this.inShootingZone = inShootingZone;
            this.ready = ready;
            this.targetRpm = targetRpm;
            this.actualRpm = actualRpm;
            this.compensationRpm = compensationRpm;
            this.nominalHoodDeg = nominalHoodDeg;
            this.compensatedHoodDeg = compensatedHoodDeg;
            this.aimGoalX = aimGoalX;
            this.aimGoalY = aimGoalY;
            this.goalForwardOffsetInches = goalForwardOffsetInches;
            this.turretAngleDeg = turretAngleDeg;
            this.turretTargetDeg = turretTargetDeg;
            this.predictedRangeInches = predictedRangeInches;
            this.tableDistanceInches = tableDistanceInches;
            this.timeOfFlightSeconds = timeOfFlightSeconds;
            this.status = status;
            this.alliance = alliance;
        }
    }

    private static final class TurretKinematics {
        final double x;
        final double y;
        final double vx;
        final double vy;

        TurretKinematics(double x, double y, double vx, double vy) {
            this.x = x;
            this.y = y;
            this.vx = vx;
            this.vy = vy;
        }
    }

    private static final class ScorePoint {
        final double x;
        final double y;
        final double distanceInches;

        ScorePoint(double x, double y, double distanceInches) {
            this.x = x;
            this.y = y;
            this.distanceInches = distanceInches;
        }
    }

    private final ShooterConstants config;
    private final Follower follower;
    private final FlywheelVelocityController flywheel;
    private final ServoHoodController hood;
    private final TurretAxis turret;
    private final Servo feedServo;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime chassisAimTimer = new ElapsedTime();
    private final ElapsedTime readyDropoutTimer = new ElapsedTime();

    private Alliance alliance = Alliance.BLUE;
    private boolean fireRequested;
    private boolean spinEnabled;
    private boolean autoAimEnabled = true;
    private boolean useAutoShootingZoneRadius;
    private boolean readyRequiresOnlyFlywheel;
    private boolean customGoalEnabled;
    private boolean customAimGoalEnabled;
    private Pose lastPose;
    private double robotVx;
    private double robotVy;
    private double robotOmega;
    private double yawTrimRadians;
    private double rpmTrim;
    private boolean manualHoodOverrideEnabled;
    private double manualHoodOverrideDeg;
    private boolean manualTurretOverrideEnabled;
    private double manualTurretOverrideRadians;
    private double customGoalX;
    private double customGoalY;
    private double customAimGoalX;
    private double customAimGoalY;
    private double lastAimGoalX;
    private double lastAimGoalY;
    private boolean lastInShootingZone;
    private ShootOnMoveCalculator.Solution lastSolution = ShootOnMoveCalculator.Solution.invalid("startup");
    private PrecisionShotTable.Entry lastNominal = new PrecisionShotTable.Entry(0.0, 0.0, 40.0);
    private double lastTableDistanceInches;
    private double lastCompensatedHoodDeg = 40.0;
    private double lastGoalForwardOffsetInches;
    private boolean filteredAimInitialized;
    private double filteredWorldYawRadians;
    private double lastChassisAimTurnCommand;
    private boolean feedGateOpen;
    private double chassisAimIntegral;
    private double lastChassisAimErrorRadians;
    private double lastChassisAimProportionalTerm;
    private double lastChassisAimIntegralTerm;
    private double lastChassisAimDerivativeTerm;
    private double lastChassisAimFeedforwardTerm;

    private PrecisionShooterSubsystem(ShooterConstants config,
                                      Follower follower,
                                      FlywheelVelocityController flywheel,
                                      ServoHoodController hood,
                                      TurretAxis turret,
                                      Servo feedServo) {
        this.config = config;
        this.follower = follower;
        this.flywheel = flywheel;
        this.hood = hood;
        this.turret = turret;
        this.feedServo = feedServo;

        if (feedServo != null) {
            feedServo.setPosition(ShooterHardwareConstants.feedClosedPosition);
        }
        loopTimer.reset();
        feedTimer.reset();
        chassisAimTimer.reset();
        readyDropoutTimer.reset();
    }

    public static PrecisionShooterSubsystem create(HardwareMap hardwareMap,
                                                   Follower follower,
                                                   ShooterConstants config) {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, ShooterHardwareConstants.leftFlywheelName);
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, ShooterHardwareConstants.rightFlywheelName);
        Servo hoodServo = hardwareMap.get(Servo.class, ShooterHardwareConstants.hoodServoName);
        Servo feedServo = null;
        TurretAxis turret = null;

        if (ShooterHardwareConstants.turretEnabled) {
            DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, ShooterHardwareConstants.turretMotorName);
            turret = new TurretAxis(turretMotor, config);
        }

        try {
            feedServo = hardwareMap.get(Servo.class, ShooterHardwareConstants.feedServoName);
        } catch (Exception ignored) {
        }
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        FlywheelVelocityController flywheel = new FlywheelVelocityController(left, right, voltageSensor, config);
        ServoHoodController hood = new ServoHoodController(hoodServo, config);
        return new PrecisionShooterSubsystem(config, follower, flywheel, hood, turret, feedServo);
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setSpinEnabled(boolean spinEnabled) {
        this.spinEnabled = spinEnabled;
    }

    public void setAutoAimEnabled(boolean autoAimEnabled) {
        this.autoAimEnabled = autoAimEnabled;
    }

    public void setUseAutoShootingZoneRadius(boolean useAutoShootingZoneRadius) {
        this.useAutoShootingZoneRadius = useAutoShootingZoneRadius;
    }

    public void setReadyRequiresOnlyFlywheel(boolean readyRequiresOnlyFlywheel) {
        this.readyRequiresOnlyFlywheel = readyRequiresOnlyFlywheel;
    }

    public void requestFire(boolean fireRequested) {
        this.fireRequested = fireRequested;
    }

    public void setGoalPosition(double goalX, double goalY) {
        customGoalEnabled = true;
        customGoalX = goalX;
        customGoalY = goalY;
    }

    public void clearGoalPositionOverride() {
        customGoalEnabled = false;
    }

    public void setAimPosition(double goalX, double goalY) {
        customAimGoalEnabled = true;
        customAimGoalX = goalX;
        customAimGoalY = goalY;
    }

    public void clearAimPositionOverride() {
        customAimGoalEnabled = false;
    }

    public void setYawTrimDegrees(double yawTrimDegrees) {
        yawTrimRadians = Math.toRadians(yawTrimDegrees);
    }

    public double getYawTrimDegrees() {
        return Math.toDegrees(yawTrimRadians);
    }

    public void setRpmTrim(double rpmTrim) {
        this.rpmTrim = rpmTrim;
    }

    public double getRpmTrim() {
        return rpmTrim;
    }

    public void setManualHoodAngleOverrideDegrees(double hoodAngleDeg) {
        manualHoodOverrideEnabled = true;
        manualHoodOverrideDeg = hoodAngleDeg;
    }

    public void clearManualHoodAngleOverride() {
        manualHoodOverrideEnabled = false;
    }

    public void setManualTurretAngleOverrideDegrees(double turretAngleDeg) {
        manualTurretOverrideEnabled = true;
        manualTurretOverrideRadians = Math.toRadians(turretAngleDeg);
    }

    public void clearManualTurretAngleOverride() {
        manualTurretOverrideEnabled = false;
    }

    public void start() {
        follower.startTeleopDrive();
        notifyPoseJump();
        feedTimer.reset();
        feedGateOpen = false;
    }

    public void notifyPoseJump() {
        lastPose = follower.getPose();
        robotVx = 0.0;
        robotVy = 0.0;
        robotOmega = 0.0;
        filteredAimInitialized = false;
        lastChassisAimTurnCommand = 0.0;
        resetChassisAimController();
        loopTimer.reset();
        chassisAimTimer.reset();
        readyDropoutTimer.reset();
    }

    public void primeHeadingLock() {
        resetChassisAimController();
        chassisAimTimer.reset();
    }

    public void update() {
        Pose pose = follower.getPose();
        updateMotionEstimate(pose);
        if (turret != null) {
            turret.updateHoming();
        }

        if (turret != null && !turret.isHomed()) {
            hood.setAngleDegrees(ShooterHardwareConstants.hoodMaxAngleDeg);
            hood.update();
            flywheel.stop();
            closeFeed();
            lastSolution = ShootOnMoveCalculator.Solution.invalid("turret homing");
            return;
        }

        double goalX = customGoalEnabled
                ? customGoalX
                : (alliance == Alliance.BLUE ? config.blueGoalXInches : config.redGoalXInches);
        double goalY = customGoalEnabled
                ? customGoalY
                : (alliance == Alliance.BLUE ? config.blueGoalYInches : config.redGoalYInches);
        double aimGoalX = customAimGoalEnabled
                ? customAimGoalX
                : (alliance == Alliance.BLUE ? config.blueHeadingAimXInches : config.redHeadingAimXInches);
        double aimGoalY = customAimGoalEnabled
                ? customAimGoalY
                : (alliance == Alliance.BLUE ? config.blueHeadingAimYInches : config.redHeadingAimYInches);
        lastAimGoalX = aimGoalX;
        lastAimGoalY = aimGoalY;

        TurretKinematics releaseTurret = computeTurretKinematics(pose, robotVx, robotVy, robotOmega);
        ScorePoint scorePoint = config.shootOnMoveEnabled
                ? computeScorePoint(releaseTurret, goalX, goalY, config.sotmPassThroughRadiusInches)
                : new ScorePoint(goalX, goalY, Math.hypot(goalX - releaseTurret.x, goalY - releaseTurret.y));
        ScorePoint aimScorePoint = config.shootOnMoveEnabled
                ? computeScorePoint(releaseTurret, aimGoalX, aimGoalY, config.sotmPassThroughRadiusInches)
                : new ScorePoint(aimGoalX, aimGoalY, Math.hypot(aimGoalX - releaseTurret.x, aimGoalY - releaseTurret.y));
        lastTableDistanceInches = scorePoint.distanceInches;
        lastGoalForwardOffsetInches = config.shootOnMoveEnabled ? config.sotmPassThroughRadiusInches : 0.0;
        double shootingZoneRadius = useAutoShootingZoneRadius
                ? ShootingZones.autoRadiusInches
                : ShootingZones.radiusInches;
        lastInShootingZone = ShootingZones.isInShootingZone(pose.getX(), pose.getY(), shootingZoneRadius);
        double targetHeightDeltaInches = config.targetHeightInches - config.shooterHeightInches;

        ShootOnMoveCalculator.Solution aimSolution;
        if (config.shootOnMoveEnabled) {
            double scoreAngleRad = Math.toRadians(config.sotmScoreAngleDeg);
            double minLaunchAngleRad = Math.min(
                    hoodCommandToLaunchAngleRadians(ShooterHardwareConstants.hoodMinAngleDeg),
                    hoodCommandToLaunchAngleRadians(ShooterHardwareConstants.hoodMaxAngleDeg)
            );
            double maxLaunchAngleRad = Math.max(
                    hoodCommandToLaunchAngleRadians(ShooterHardwareConstants.hoodMinAngleDeg),
                    hoodCommandToLaunchAngleRadians(ShooterHardwareConstants.hoodMaxAngleDeg)
            );
            ShootOnMoveCalculator.Solution nominalSolution = ShootOnMoveCalculator.solveStationary(
                    releaseTurret.x,
                    releaseTurret.y,
                    scorePoint.x,
                    scorePoint.y,
                    targetHeightDeltaInches,
                    scoreAngleRad,
                    ShooterConstants.GRAVITY_INCHES_PER_SECOND_SQUARED
            );
            lastNominal = nominalSolution.valid
                    ? new PrecisionShotTable.Entry(
                    lastTableDistanceInches,
                    launchSpeedToRpm(nominalSolution.launchSpeedIps),
                    launchAngleToHoodCommandDegrees(nominalSolution.hoodAngleRad)
            )
                    : new PrecisionShotTable.Entry(lastTableDistanceInches, 0.0, ShooterHardwareConstants.hoodMaxAngleDeg);

            lastSolution = ShootOnMoveCalculator.solveCompensated(
                    releaseTurret.x,
                    releaseTurret.y,
                    scorePoint.x,
                    scorePoint.y,
                    targetHeightDeltaInches,
                    releaseTurret.vx,
                    releaseTurret.vy,
                    scoreAngleRad,
                    minLaunchAngleRad,
                    maxLaunchAngleRad,
                    ShooterConstants.GRAVITY_INCHES_PER_SECOND_SQUARED
            );
            aimSolution = ShootOnMoveCalculator.solveCompensated(
                    releaseTurret.x,
                    releaseTurret.y,
                    aimScorePoint.x,
                    aimScorePoint.y,
                    targetHeightDeltaInches,
                    releaseTurret.vx,
                    releaseTurret.vy,
                    scoreAngleRad,
                    minLaunchAngleRad,
                    maxLaunchAngleRad,
                    ShooterConstants.GRAVITY_INCHES_PER_SECOND_SQUARED
            );
            lastCompensatedHoodDeg = lastSolution.valid
                    ? launchAngleToHoodCommandDegrees(lastSolution.hoodAngleRad)
                    : lastNominal.hoodAngleDeg;
        } else {
            lastNominal = LUTConstants.currentTable().sample(lastTableDistanceInches);
            lastSolution = buildLookupSolution(releaseTurret, goalX, goalY, lastNominal, true, "lookup table aim lead");
            aimSolution = buildLookupSolution(releaseTurret, aimGoalX, aimGoalY, lastNominal, true, "lookup table aim lead");
            lastCompensatedHoodDeg = lastNominal.hoodAngleDeg;
        }

        double targetRpm = 0.0;
        if (spinEnabled) {
            double baseRpm = config.shootOnMoveEnabled && lastSolution.valid
                    ? launchSpeedToRpm(lastSolution.launchSpeedIps)
                    : lastNominal.targetRpm;
            targetRpm = Math.max(0.0, baseRpm + rpmTrim);
        }
        flywheel.setTargetRpm(targetRpm);
        flywheel.update(config.flywheelIntegralLimit);

        double desiredAimWorldYawRadians = aimSolution.valid ? aimSolution.worldYawRad : lastSolution.worldYawRad;
        updateFilteredAimTarget(aimSolution.valid || lastSolution.valid, desiredAimWorldYawRadians);

        if (autoAimEnabled) {
            hood.setAngleDegrees(manualHoodOverrideEnabled ? manualHoodOverrideDeg : lastCompensatedHoodDeg);
        }
        if (turret != null) {
            if (manualTurretOverrideEnabled) {
                turret.setTargetAngleRadians(manualTurretOverrideRadians);
            } else if (autoAimEnabled && lastSolution.valid) {
                turret.setTargetAngleRadians(
                        ShooterMath.normalizeRadians(getFilteredWorldYawRadians() + yawTrimRadians - pose.getHeading())
                );
            }
        }

        hood.update();
        if (turret != null) {
            turret.updateTracking();
        }

        boolean readyToOpenFeed = isReadyToShoot();
        boolean readyToContinueFeeding = isReadyToContinueFeeding();

        if (readyToContinueFeeding) {
            readyDropoutTimer.reset();
        }

        if (!fireRequested) {
            closeFeed();
        } else if (!feedGateOpen) {
            if (readyToOpenFeed) {
                openFeed();
            }
        } else if (!readyToContinueFeeding
                && !shouldHoldFeedGateOpenDuringFarZoneSpeedDip()
                && readyDropoutTimer.seconds() >= config.feedReadyDropoutGraceSeconds) {
            closeFeed();
        }
    }

    public TelemetrySnapshot snapshot() {
        return new TelemetrySnapshot(
                turret == null || turret.isHomed(),
                lastSolution.valid,
                lastInShootingZone,
                isReadyToShoot(),
                flywheel.getTargetRpm(),
                flywheel.getMeasuredRpm(),
                getCompensationRpm(),
                lastNominal.hoodAngleDeg,
                lastCompensatedHoodDeg,
                lastAimGoalX,
                lastAimGoalY,
                lastGoalForwardOffsetInches,
                Math.toDegrees(turret == null ? 0.0 : turret.getCurrentAngleRadians()),
                Math.toDegrees(turret == null ? 0.0 : turret.getTargetAngleRadians()),
                lastSolution.rangeInches,
                lastTableDistanceInches,
                lastSolution.flightTimeSeconds,
                lastSolution.reason,
                alliance
        );
    }

    public boolean isReadyToShootNow() {
        return isReadyToShoot();
    }

    public boolean isTurretEnabled() {
        return turret != null;
    }

    public boolean isTurretHomed() {
        return turret == null || turret.isHomed();
    }

    public boolean isFeedGateOpen() {
        return feedGateOpen;
    }

    public boolean isFireRequested() {
        return fireRequested;
    }

    public boolean isSpinEnabled() {
        return spinEnabled;
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    public double getCompensationRpm() {
        return flywheel.getTargetRpm();
    }

    public double getFlywheelErrorRpm() {
        return flywheel.getLastErrorRpm();
    }

    public double getFlywheelControlPower() {
        return flywheel.getLastOutputPower();
    }

    public double getBatteryVoltage() {
        return flywheel.getLastBatteryVoltage();
    }

    public boolean shouldUseChassisHeadingLock() {
        return turret == null && config.lockChassisHeadingWhenTurretDisabled;
    }

    public double getChassisHeadingErrorRadians() {
        if (!lastSolution.valid || !filteredAimInitialized) {
            return 0.0;
        }
        return ShooterMath.normalizeRadians(getFilteredWorldYawRadians() - follower.getPose().getHeading());
    }

    public double getAdjustedChassisHeadingErrorRadians() {
        if (!lastSolution.valid || !filteredAimInitialized) {
            return 0.0;
        }
        return ShooterMath.normalizeRadians(getFilteredWorldYawRadians() + yawTrimRadians - follower.getPose().getHeading());
    }

    public double getChassisAimTurnCommand() {
        if (!shouldUseChassisHeadingLock() || !lastSolution.valid) {
            resetChassisAimController();
            return 0.0;
        }
        double dt = Math.max(1e-3, chassisAimTimer.seconds());
        chassisAimTimer.reset();
        double error = getAdjustedChassisHeadingErrorRadians();
        if (Math.abs(error) <= CHASSIS_AIM_SETTLED_ERROR_RADIANS
                && Math.abs(robotOmega) <= CHASSIS_AIM_SETTLED_OMEGA_RADIANS) {
            resetChassisAimController();
            return 0.0;
        }

        if (lastChassisAimErrorRadians != 0.0 && error * lastChassisAimErrorRadians < 0.0) {
            chassisAimIntegral = 0.0;
        }

        lastChassisAimProportionalTerm = error * config.chassisAimKp;
        lastChassisAimDerivativeTerm = -robotOmega * config.chassisAimKd;
        lastChassisAimFeedforwardTerm = Math.abs(error) > CHASSIS_AIM_SETTLED_ERROR_RADIANS
                ? Math.signum(error) * config.chassisAimKf
                : 0.0;

        if (config.chassisAimKi != 0.0) {
            double maxIntegral = CHASSIS_AIM_MAX_INTEGRAL_TERM / Math.abs(config.chassisAimKi);
            double candidateIntegral = ShooterMath.clamp(chassisAimIntegral + error * dt, -maxIntegral, maxIntegral);
            double candidateIntegralTerm = candidateIntegral * config.chassisAimKi;
            double unclampedCandidateCommand = lastChassisAimProportionalTerm
                    + candidateIntegralTerm
                    + lastChassisAimDerivativeTerm
                    + lastChassisAimFeedforwardTerm;
            boolean candidatePushesFurtherIntoSaturation =
                    Math.abs(unclampedCandidateCommand) > CHASSIS_AIM_COMMAND_LIMIT
                            && Math.signum(unclampedCandidateCommand) == Math.signum(error);
            if (!candidatePushesFurtherIntoSaturation) {
                chassisAimIntegral = candidateIntegral;
            }
        } else {
            chassisAimIntegral = 0.0;
        }
        lastChassisAimIntegralTerm = chassisAimIntegral * config.chassisAimKi;

        double command = lastChassisAimProportionalTerm
                + lastChassisAimIntegralTerm
                + lastChassisAimDerivativeTerm
                + lastChassisAimFeedforwardTerm;
        command = ShooterMath.clamp(command, -CHASSIS_AIM_COMMAND_LIMIT, CHASSIS_AIM_COMMAND_LIMIT);
        lastChassisAimErrorRadians = error;
        lastChassisAimTurnCommand = command;
        return command;
    }

    public double getLastChassisAimTurnCommand() {
        return lastChassisAimTurnCommand;
    }

    public double getLastChassisAimProportionalTerm() {
        return lastChassisAimProportionalTerm;
    }

    public double getLastChassisAimIntegralTerm() {
        return lastChassisAimIntegralTerm;
    }

    public double getLastChassisAimDerivativeTerm() {
        return lastChassisAimDerivativeTerm;
    }

    public double getLastChassisAimFeedforwardTerm() {
        return lastChassisAimFeedforwardTerm;
    }

    public double getRobotOmegaRadiansPerSecond() {
        return robotOmega;
    }

    public double getTargetRpm() {
        return flywheel.getTargetRpm();
    }

    public double getActualRpm() {
        return flywheel.getMeasuredRpm();
    }

    public double getTurretAngleDegrees() {
        return Math.toDegrees(turret == null ? 0.0 : turret.getCurrentAngleRadians());
    }

    public double getTurretTargetDegrees() {
        return Math.toDegrees(turret == null ? 0.0 : turret.getTargetAngleRadians());
    }

    public double getTurretErrorDegrees() {
        return Math.toDegrees(turret == null ? 0.0 : turret.getLastAngleErrorRadians());
    }

    public double getTurretCommandPower() {
        return turret == null ? 0.0 : turret.getLastCommandPower();
    }

    public double getTurretSoftLimitScale() {
        return turret == null ? 1.0 : turret.getLastSoftLimitScale();
    }

    public boolean shouldNudgeChassisForTurretDeadZone() {
        return turret != null
                && turret.isHomed()
                && autoAimEnabled
                && fireRequested
                && lastInShootingZone
                && lastSolution.valid
                && turret.isRequestedAngleInForwardDeadZone();
    }

    public boolean isTurretRequestedAngleInForwardDeadZone() {
        return turret != null
                && turret.isHomed()
                && turret.isRequestedAngleInForwardDeadZone();
    }

    public double getTurretDeadZoneNudgeTurnCommand() {
        if (!shouldNudgeChassisForTurretDeadZone()) {
            return 0.0;
        }

        return turret.getDeadZoneEscapeChassisTurnDirection()
                * config.turretDeadZoneNudgeTurnCommand;
    }

    public double getTurretCurrentAmps() {
        return turret == null ? 0.0 : turret.getMotorCurrentAmps();
    }

    public double getTurretVelocityTicksPerSecond() {
        return turret == null ? 0.0 : turret.getLastVelocityTicksPerSecond();
    }

    public int getTurretLeftStopTicks() {
        return turret == null ? 0 : turret.getLeftStopTicks();
    }

    public int getTurretRightStopTicks() {
        return turret == null ? 0 : turret.getRightStopTicks();
    }

    public String getTurretHomeState() {
        return turret == null ? "DISABLED" : turret.getHomeStateName();
    }

    public boolean restoreTurretHome(android.content.Context context) {
        if (turret == null) {
            return false;
        }

        TurretHomeHandoff.Calibration calibration = TurretHomeHandoff.consumeCalibration(context);
        return calibration != null && turret.loadHomeCalibration(
                calibration.leftStopTicks,
                calibration.rightStopTicks
        );
    }

    public void saveTurretHome(android.content.Context context) {
        if (turret == null || !turret.isHomed()) {
            return;
        }

        TurretHomeHandoff.saveCalibration(
                context,
                turret.getLeftStopTicks(),
                turret.getRightStopTicks()
        );
    }

    public void requestTurretRehome() {
        if (turret != null) {
            turret.restartHoming();
        }
    }

    public double getTurretMinTargetDegrees() {
        return Math.toDegrees(turret == null ? 0.0 : turret.getMinimumCommandedAngleRadians());
    }

    public double getTurretMaxTargetDegrees() {
        return Math.toDegrees(turret == null ? 0.0 : turret.getMaximumCommandedAngleRadians());
    }

    private boolean isReadyToShoot() {
        return isReadyToShoot(false);
    }

    private boolean isReadyToContinueFeeding() {
        return isReadyToShoot(true);
    }

    private boolean shouldHoldFeedGateOpenDuringFarZoneSpeedDip() {
        return fireRequested
                && spinEnabled
                && feedGateOpen
                && lastInShootingZone
                && lastSolution.valid
                && lastTableDistanceInches >= ShooterConstants.farZoneFeedPauseDistanceInches
                && hood.isSettled()
                && isAimAligned(true);
    }

    private boolean isReadyToShoot(boolean sustainShot) {
        if (!spinEnabled) {
            return false;
        }
        double flywheelToleranceRpm = sustainShot
                ? config.flywheelSustainToleranceRpm
                : config.flywheelReadyToleranceRpm;
        double flywheelToleranceFraction = sustainShot
                ? config.flywheelSustainToleranceFraction
                : config.flywheelReadyToleranceFraction;
        if (readyRequiresOnlyFlywheel) {
            return flywheel.atSpeed(flywheelToleranceRpm, flywheelToleranceFraction);
        }
        return (turret == null || turret.isHomed())
                && lastInShootingZone
                && lastSolution.valid
                && flywheel.atSpeed(flywheelToleranceRpm, flywheelToleranceFraction)
                && hood.isSettled()
                && isAimAligned(sustainShot);
    }

    private boolean isAimAligned(boolean sustainShot) {
        if (turret != null) {
            double toleranceDeg = sustainShot
                    ? config.turretShotSustainToleranceDeg
                    : config.turretShotReadyToleranceDeg;
            return turret.atTarget(Math.toRadians(toleranceDeg));
        }
        double headingToleranceDeg = sustainShot
                ? config.shotSustainHeadingToleranceDeg
                : config.shotReadyHeadingToleranceDeg;
        double maxOmegaDegPerSecond = sustainShot
                ? config.shotSustainMaxOmegaDegPerSecond
                : config.shotReadyMaxOmegaDegPerSecond;
        return Math.abs(getAdjustedChassisHeadingErrorRadians()) <= Math.toRadians(headingToleranceDeg)
                && Math.abs(robotOmega) <= Math.toRadians(maxOmegaDegPerSecond);
    }

    public boolean isInShootingZone() {
        return lastInShootingZone;
    }

    private TurretKinematics computeTurretKinematics(Pose pose,
                                                     double chassisVx,
                                                     double chassisVy,
                                                     double chassisOmega) {
        double heading = pose.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double offsetWorldX = ShooterHardwareConstants.turretOffsetForwardInches * cos
                - ShooterHardwareConstants.turretOffsetLeftInches * sin;
        double offsetWorldY = ShooterHardwareConstants.turretOffsetForwardInches * sin
                + ShooterHardwareConstants.turretOffsetLeftInches * cos;

        // Rigid-body velocity of the turret pivot = chassis translation + rotation-induced point velocity.
        double turretVx = chassisVx - chassisOmega * offsetWorldY;
        double turretVy = chassisVy + chassisOmega * offsetWorldX;

        return new TurretKinematics(
                pose.getX() + offsetWorldX,
                pose.getY() + offsetWorldY,
                turretVx,
                turretVy
        );
    }

    private ShootOnMoveCalculator.Solution buildLookupSolution(TurretKinematics turretKinematics,
                                                               double targetX,
                                                               double targetY,
                                                               PrecisionShotTable.Entry tableEntry,
                                                               boolean leadAimWithRobotVelocity,
                                                               String reason) {
        double dx = targetX - turretKinematics.x;
        double dy = targetY - turretKinematics.y;
        double range = Math.hypot(dx, dy);
        if (range <= 1e-6) {
            return ShootOnMoveCalculator.Solution.invalid("target unreachable");
        }

        double hoodAngleRad = hoodCommandToLaunchAngleRadians(tableEntry.hoodAngleDeg);
        double launchSpeedIps = ShooterMath.solveLaunchSpeed(
                range,
                config.targetHeightInches - config.shooterHeightInches,
                hoodAngleRad,
                ShooterConstants.GRAVITY_INCHES_PER_SECOND_SQUARED
        );
        if (Double.isNaN(launchSpeedIps) || launchSpeedIps <= 1e-6) {
            launchSpeedIps = 0.0;
        }

        double aimFlightTimeSeconds = estimateLookupAimFlightTimeSeconds(range);
        double worldYawRad = Math.atan2(dy, dx);
        if (leadAimWithRobotVelocity && aimFlightTimeSeconds > 1e-6) {
            double aimVx = dx / aimFlightTimeSeconds - turretKinematics.vx;
            double aimVy = dy / aimFlightTimeSeconds - turretKinematics.vy;
            if (Math.hypot(aimVx, aimVy) > 1e-6) {
                worldYawRad = Math.atan2(aimVy, aimVx);
            }
        }

        return new ShootOnMoveCalculator.Solution(
                true,
                hoodAngleRad,
                worldYawRad,
                aimFlightTimeSeconds,
                launchSpeedIps,
                range,
                hoodAngleRad,
                launchSpeedIps,
                reason
        );
    }

    private double estimateLookupAimFlightTimeSeconds(double rangeInches) {
        double estimatedSeconds = config.lookupAimFlightTimeInterceptSeconds
                + rangeInches * config.lookupAimFlightTimeSecondsPerInch;
        return ShooterMath.clamp(
                estimatedSeconds,
                config.lookupAimFlightTimeMinSeconds,
                config.lookupAimFlightTimeMaxSeconds
        );
    }

    private ScorePoint computeScorePoint(TurretKinematics turretKinematics,
                                         double goalX,
                                         double goalY,
                                         double forwardOffsetInches) {
        double dx = goalX - turretKinematics.x;
        double dy = goalY - turretKinematics.y;
        double goalDistance = Math.hypot(dx, dy);
        if (goalDistance <= 1e-6) {
            return new ScorePoint(goalX, goalY, 0.0);
        }

        double scoreDistance = Math.max(0.0, goalDistance - forwardOffsetInches);
        double unitX = dx / goalDistance;
        double unitY = dy / goalDistance;
        return new ScorePoint(
                turretKinematics.x + unitX * scoreDistance,
                turretKinematics.y + unitY * scoreDistance,
                scoreDistance
        );
    }

    private double hoodCommandToLaunchAngleRadians(double hoodCommandDeg) {
        return Math.toRadians(
                ShooterHardwareConstants.hoodMinAngleDeg
                        + ShooterHardwareConstants.hoodMaxAngleDeg
                        - hoodCommandDeg
        );
    }

    private double launchAngleToHoodCommandDegrees(double launchAngleRad) {
        return ShooterMath.clamp(
                ShooterHardwareConstants.hoodMinAngleDeg
                        + ShooterHardwareConstants.hoodMaxAngleDeg
                        - Math.toDegrees(launchAngleRad),
                ShooterHardwareConstants.hoodMinAngleDeg,
                ShooterHardwareConstants.hoodMaxAngleDeg
        );
    }

    private double launchSpeedToRpm(double launchSpeedIps) {
        return Math.max(
                0.0,
                config.sotmLaunchSpeedToRpmSlope * launchSpeedIps + config.sotmLaunchSpeedToRpmIntercept
        );
    }

    private void updateMotionEstimate(Pose pose) {
        double dt = Math.max(1e-3, loopTimer.seconds());
        loopTimer.reset();
        if (lastPose == null) {
            lastPose = pose;
            robotVx = 0.0;
            robotVy = 0.0;
            robotOmega = 0.0;
            return;
        }

        try {
            Vector velocity = follower.getVelocity();
            if (velocity != null) {
                robotVx = velocity.getXComponent();
                robotVy = velocity.getYComponent();
            } else {
                robotVx = (pose.getX() - lastPose.getX()) / dt;
                robotVy = (pose.getY() - lastPose.getY()) / dt;
            }
        } catch (Exception ignored) {
            robotVx = (pose.getX() - lastPose.getX()) / dt;
            robotVy = (pose.getY() - lastPose.getY()) / dt;
        }

        robotOmega = ShooterMath.normalizeRadians(pose.getHeading() - lastPose.getHeading()) / dt;
        lastPose = pose;
    }

    private void openFeed() {
        if (feedServo == null) {
            return;
        }
        feedServo.setPosition(ShooterHardwareConstants.feedOpenPosition);
        feedTimer.reset();
        readyDropoutTimer.reset();
        feedGateOpen = true;
    }

    private void closeFeed() {
        if (feedServo != null) {
            feedServo.setPosition(ShooterHardwareConstants.feedClosedPosition);
        }
        readyDropoutTimer.reset();
        feedGateOpen = false;
    }

    private void updateFilteredAimTarget(boolean targetValid, double desiredWorldYawRadians) {
        if (!targetValid) {
            filteredAimInitialized = false;
            return;
        }
        filteredWorldYawRadians = desiredWorldYawRadians;
        filteredAimInitialized = true;
    }

    private double getFilteredWorldYawRadians() {
        return filteredWorldYawRadians;
    }

    private void resetChassisAimController() {
        chassisAimIntegral = 0.0;
        lastChassisAimErrorRadians = 0.0;
        lastChassisAimTurnCommand = 0.0;
        lastChassisAimProportionalTerm = 0.0;
        lastChassisAimIntegralTerm = 0.0;
        lastChassisAimDerivativeTerm = 0.0;
        lastChassisAimFeedforwardTerm = 0.0;
    }
}
