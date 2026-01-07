package org.firstinspires.ftc.teamcode.Decode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.708987)
            .forwardZeroPowerAcceleration(-34.682475)
            .lateralZeroPowerAcceleration(-64.2)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2/*0.22*/, 0, 0.035, 0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.035, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.35, 0, 0.11, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.35, 0, 0.11, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.022, 0, 0.0012, 0.6, 0.01))
            .centripetalScaling(0.1)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.5)
            .rightFrontMotorName("fR")
            .rightRearMotorName("bR")
            .leftRearMotorName("bL")
            .leftFrontMotorName("fL")
            .leftFrontMotorDirection(DcMotor.Direction.FORWARD)
            .leftRearMotorDirection(DcMotor.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(80.17470280204233)
            .yVelocity(59.89506278451033);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7.75)
            .strafePodX(-7.625)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("follower")
            // default resolution .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .customEncoderResolution(0.5260988946574804)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
