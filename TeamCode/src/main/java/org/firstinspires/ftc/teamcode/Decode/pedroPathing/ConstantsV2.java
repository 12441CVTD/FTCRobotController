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

public class ConstantsV2 {
      public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.00)
            .forwardZeroPowerAcceleration(-42.632832476913734)
            .lateralZeroPowerAcceleration(-75.11499624472049)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.009, 0.01))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.22, 0, 0.035, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.15, 0.01))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.35, 0, 0.11, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.001, 0.6, 0.01))
//            .centripetalScaling(0.1)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
              ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.75)
            .rightFrontMotorName("fR")
            .rightRearMotorName("bR")
            .leftRearMotorName("bL")
            .leftFrontMotorName("fL")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(56.929744690422)
            .yVelocity(40.41952130175012)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.8125)
            .strafePodX(-5.5625)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("follower")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}