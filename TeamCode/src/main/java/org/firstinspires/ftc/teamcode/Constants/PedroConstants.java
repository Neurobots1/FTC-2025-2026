package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PedroConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            // Predictive braking setup: keep heading PIDF, predictive braking, and
            // disable centripetal correction because predictive braking naturally handles it.
            .mass(13.6078)
            .centripetalScaling(0)
            .automaticHoldEnd(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.85 ,0,0.1,0.01))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.04, 0.0016));


    public static MecanumConstants driveConstants = new MecanumConstants()
            //motor
            .rightFrontMotorName(HardwareMapConstants.RIGHT_FRONT_DRIVE_MOTOR)
            .rightRearMotorName(HardwareMapConstants.RIGHT_REAR_DRIVE_MOTOR)
            .leftRearMotorName(HardwareMapConstants.LEFT_REAR_DRIVE_MOTOR)
            .leftFrontMotorName(HardwareMapConstants.LEFT_FRONT_DRIVE_MOTOR)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //Drive constant
            .maxPower(1)
            .useVoltageCompensation(true)
            .nominalVoltage(13)
            .xVelocity(74.2403)
            .yVelocity(58.7281);





    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.959645669)
            .strafePodX(-1.0837192913)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(HardwareMapConstants.PINPOINT)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);





    public static PathConstraints pathConstraints = new PathConstraints(0.95, 50, 4, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

    }


}
