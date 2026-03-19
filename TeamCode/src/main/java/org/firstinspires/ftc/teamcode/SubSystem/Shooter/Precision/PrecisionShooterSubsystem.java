package org.firstinspires.ftc.teamcode.SubSystem.Shooter.Precision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.PrecisionShooterConfig;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter.ShootingZones;

public final class PrecisionShooterSubsystem {

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
        public final double nominalHoodDeg;
        public final double compensatedHoodDeg;
        public final double turretAngleDeg;
        public final double turretTargetDeg;
        public final double predictedRangeInches;
        public final double timeOfFlightSeconds;
        public final String status;
        public final Alliance alliance;

        TelemetrySnapshot(boolean homed,
                          boolean solutionValid,
                          boolean inShootingZone,
                          boolean ready,
                          double targetRpm,
                          double actualRpm,
                          double nominalHoodDeg,
                          double compensatedHoodDeg,
                          double turretAngleDeg,
                          double turretTargetDeg,
                          double predictedRangeInches,
                          double timeOfFlightSeconds,
                          String status,
                          Alliance alliance) {
            this.homed = homed;
            this.solutionValid = solutionValid;
            this.inShootingZone = inShootingZone;
            this.ready = ready;
            this.targetRpm = targetRpm;
            this.actualRpm = actualRpm;
            this.nominalHoodDeg = nominalHoodDeg;
            this.compensatedHoodDeg = compensatedHoodDeg;
            this.turretAngleDeg = turretAngleDeg;
            this.turretTargetDeg = turretTargetDeg;
            this.predictedRangeInches = predictedRangeInches;
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

    private final PrecisionShooterConfig config;
    private final Follower follower;
    private final FlywheelVelocityController flywheel;
    private final ServoHoodController hood;
    private final TurretAxis turret;
    private final Servo feedServo;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    private Alliance alliance = Alliance.BLUE;
    private boolean fireRequested;
    private boolean spinEnabled;
    private boolean autoAimEnabled = true;
    private boolean customGoalEnabled;
    private Pose lastPose;
    private double robotVx;
    private double robotVy;
    private double robotOmega;
    private double customGoalX;
    private double customGoalY;
    private boolean lastInShootingZone;
    private BallisticAimSolver.Solution lastSolution = BallisticAimSolver.Solution.invalid("startup");
    private PrecisionShotTable.Entry lastNominal = new PrecisionShotTable.Entry(0.0, 0.0, 40.0);

    private PrecisionShooterSubsystem(PrecisionShooterConfig config,
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
            feedServo.setPosition(config.feedClosedPosition);
        }
        loopTimer.reset();
        feedTimer.reset();
    }

    public static PrecisionShooterSubsystem create(HardwareMap hardwareMap,
                                                   Follower follower,
                                                   PrecisionShooterConfig config) {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, config.leftFlywheelName);
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, config.rightFlywheelName);
        Servo hoodServo = hardwareMap.get(Servo.class, config.hoodServoName);
        Servo feedServo = null;
        TurretAxis turret = null;

        if (config.turretEnabled) {
            DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, config.turretMotorName);
            turret = new TurretAxis(turretMotor, config);
        }

        try {
            feedServo = hardwareMap.get(Servo.class, config.feedServoName);
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

    public void start() {
        follower.startTeleopDrive();
        notifyPoseJump();
        feedTimer.reset();
    }

    public void notifyPoseJump() {
        lastPose = follower.getPose();
        robotVx = 0.0;
        robotVy = 0.0;
        robotOmega = 0.0;
        loopTimer.reset();
    }

    public void update() {
        Pose pose = follower.getPose();
        updateMotionEstimate(pose);
        if (turret != null) {
            turret.updateHoming();
        }

        if (turret != null && !turret.isHomed()) {
            hood.setAngleDegrees(config.hoodMaxAngleDeg);
            hood.update();
            flywheel.stop();
            closeFeed();
            lastSolution = BallisticAimSolver.Solution.invalid("turret homing");
            return;
        }

        double goalX = customGoalEnabled
                ? customGoalX
                : (alliance == Alliance.BLUE ? config.blueGoalXInches : config.redGoalXInches);
        double goalY = customGoalEnabled
                ? customGoalY
                : (alliance == Alliance.BLUE ? config.blueGoalYInches : config.redGoalYInches);

        Pose releasePose = predictReleasePose(pose, config.shotReleaseLatencySeconds);
        TurretKinematics releaseTurret = computeTurretKinematics(releasePose);
        lastInShootingZone = ShootingZones.isInShootingZone(releasePose.getX(), releasePose.getY());
        double distance = Math.hypot(goalX - releaseTurret.x, goalY - releaseTurret.y);
        lastNominal = PrecisionShooterConfig.currentTable().sample(distance);

        double targetRpm = spinEnabled ? lastNominal.targetRpm : 0.0;
        flywheel.setTargetRpm(targetRpm);
        flywheel.update(5000.0, config.nominalBatteryVoltage, config.flywheelIntegralLimit);

        double effectiveRpm = Math.max(flywheel.getMeasuredRpm(), targetRpm * config.minCompensationRpmFraction);
        lastSolution = solveMovingShot(releasePose, releaseTurret, goalX, goalY, targetRpm, effectiveRpm, lastNominal);

        if (lastSolution.valid && autoAimEnabled) {
            hood.setAngleDegrees(Math.toDegrees(lastSolution.hoodAngleRad));
            if (turret != null) {
                turret.setTargetAngleRadians(
                        ShooterMath.normalizeRadians(lastSolution.worldYawRad - releasePose.getHeading())
                );
            }
        }

        hood.update();
        if (turret != null) {
            turret.updateTracking();
        }

        boolean ready = isReadyToShoot();
        if (fireRequested && ready) {
            openFeed();
        } else if (!fireRequested || feedTimer.seconds() >= config.feedActuationSeconds) {
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
                lastNominal.hoodAngleDeg,
                Math.toDegrees(lastSolution.hoodAngleRad),
                Math.toDegrees(turret == null ? 0.0 : turret.getCurrentAngleRadians()),
                Math.toDegrees(turret == null ? 0.0 : turret.getTargetAngleRadians()),
                lastSolution.rangeInches,
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

    public boolean shouldUseChassisHeadingLock() {
        return turret == null && config.lockChassisHeadingWhenTurretDisabled;
    }

    public double getChassisAimManualOverrideThreshold() {
        return config.chassisAimManualOverrideThreshold;
    }

    public double getChassisHeadingErrorRadians() {
        if (!lastSolution.valid) {
            return 0.0;
        }
        return ShooterMath.normalizeRadians(lastSolution.worldYawRad - follower.getPose().getHeading());
    }

    public double getChassisAimTurnCommand() {
        if (!shouldUseChassisHeadingLock() || !lastSolution.valid) {
            return 0.0;
        }
        double command = getChassisHeadingErrorRadians() * config.chassisAimKp;
        return ShooterMath.clamp(command, -config.chassisAimMaxTurnCommand, config.chassisAimMaxTurnCommand);
    }

    public double getTargetRpm() {
        return flywheel.getTargetRpm();
    }

    public double getActualRpm() {
        return flywheel.getMeasuredRpm();
    }

    private BallisticAimSolver.Solution solveMovingShot(Pose releasePose,
                                                        TurretKinematics releaseTurret,
                                                        double goalX,
                                                        double goalY,
                                                        double targetRpm,
                                                        double actualRpm,
                                                        PrecisionShotTable.Entry nominal) {
        if (targetRpm <= 1.0) {
            return BallisticAimSolver.Solution.invalid("spin disabled");
        }

        double nominalSpeed = ShooterMath.solveLaunchSpeed(
                nominal.distanceInches,
                config.targetHeightInches - config.shooterHeightInches,
                Math.toRadians(nominal.hoodAngleDeg),
                PrecisionShooterConfig.GRAVITY_INCHES_PER_SECOND_SQUARED
        );
        if (Double.isNaN(nominalSpeed)) {
            return BallisticAimSolver.Solution.invalid("nominal speed invalid");
        }

        double actualSpeed = nominalSpeed * (actualRpm / targetRpm);
        return BallisticAimSolver.solve(
                releaseTurret.x,
                releaseTurret.y,
                goalX,
                goalY,
                config.targetHeightInches - config.shooterHeightInches,
                releaseTurret.vx,
                releaseTurret.vy,
                actualSpeed,
                Math.toRadians(config.hoodMinAngleDeg),
                Math.toRadians(config.hoodMaxAngleDeg),
                PrecisionShooterConfig.GRAVITY_INCHES_PER_SECOND_SQUARED
        );
    }

    private boolean isReadyToShoot() {
        return (turret == null || turret.isHomed())
                && lastInShootingZone
                && lastSolution.valid
                && flywheel.atSpeed(config.flywheelReadyToleranceRpm)
                && hood.isSettled()
                && isAimAligned();
    }

    private boolean isAimAligned() {
        if (turret != null) {
            return turret.atTarget();
        }
        return Math.abs(getChassisHeadingErrorRadians()) <= Math.toRadians(config.chassisAimToleranceDeg);
    }

    public boolean isInShootingZone() {
        return lastInShootingZone;
    }

    private Pose predictReleasePose(Pose pose, double lookaheadSeconds) {
        return new Pose(
                pose.getX() + robotVx * lookaheadSeconds,
                pose.getY() + robotVy * lookaheadSeconds,
                pose.getHeading() + robotOmega * lookaheadSeconds
        );
    }

    private TurretKinematics computeTurretKinematics(Pose pose) {
        double heading = pose.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double offsetWorldX = config.turretOffsetForwardInches * cos
                - config.turretOffsetLeftInches * sin;
        double offsetWorldY = config.turretOffsetForwardInches * sin
                + config.turretOffsetLeftInches * cos;

        // Rigid-body velocity of the turret pivot = chassis translation + rotation-induced point velocity.
        double turretVx = robotVx - robotOmega * offsetWorldY;
        double turretVy = robotVy + robotOmega * offsetWorldX;

        return new TurretKinematics(
                pose.getX() + offsetWorldX,
                pose.getY() + offsetWorldY,
                turretVx,
                turretVy
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
        feedServo.setPosition(config.feedOpenPosition);
        if (feedTimer.seconds() >= config.feedActuationSeconds || feedTimer.seconds() == 0.0) {
            feedTimer.reset();
        }
    }

    private void closeFeed() {
        if (feedServo != null) {
            feedServo.setPosition(config.feedClosedPosition);
        }
    }
}
