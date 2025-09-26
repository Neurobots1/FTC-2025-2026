package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            //Follower Constant
            .mass(16)
            .forwardZeroPowerAcceleration(-25.7718)
            .lateralZeroPowerAcceleration(-72.4386)
            .centripetalScaling(0.000365)
            .automaticHoldEnd(true)

    //PIDF
            .translationalPIDFCoefficients(new PIDFCoefficients(0.18,0,0.01,0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5 ,0,0.1,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008,0,0,0.6,0))
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false);



    public static MecanumConstants driveConstants = new MecanumConstants()
            //motor
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //Drive constant
            .maxPower(1)
            .useVoltageCompensation(true)
            .nominalVoltage(13.3)
            .xVelocity(75.6121)
            .yVelocity(55.8394);





    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("sensor_otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(1.9081, 6, 0))
            .linearScalar(0.9763)
            .angularScalar(0.9795);





    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 4, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .OTOSLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
