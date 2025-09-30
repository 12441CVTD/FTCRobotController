package org.firstinspires.ftc.teamcode.Decode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;

@Autonomous(name="ExitBox", group="Decode")

public class DecodeAuto extends LinearOpMode {

    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;

    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();

    public void runOpMode() {

        Pose2d beginPose = new Pose2d(20, -60, Math.toRadians(90));

        // Initialize the drive system variables.
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(0, -31.7))
                        .waitSeconds(0.4)
                        .strafeTo(new Vector2d(0, -34.3))
                        .strafeTo(new Vector2d(50.4 , -38))
                        .strafeTo(new Vector2d(50, -36.5))
                        .waitSeconds(0.6)
                        .strafeToLinearHeading(new Vector2d(48, -47), Math.toRadians(270.0000000001))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(48, -10))
                        .strafeTo(new Vector2d(60, -10))
                        .waitSeconds(0.2)
                        .strafeTo(new Vector2d(58, -52))
                        .waitSeconds(0.6)
                        .strafeToLinearHeading(new Vector2d(3, -38.5), Math.toRadians(89.9))
                        .strafeTo(new Vector2d(3, -31))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(58, -52))

                        //pause?
                        //.strafeTo(new Vector2d(0, -40))
                        //.strafeToLinearHeading(new Vector2d(53, -24), Math.toRadians(-0))
                        //.strafeToLinearHeading(new Vector2d(50, -48), Math.toRadians(270))
                        //.strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(89.7))
                        //.strafeTo(new Vector2d(0, -33))
                        //pause?
                        //  .strafeToLinearHeading(new Vector2d(50, -55), Math.toRadians(270))
                        //  .strafeTo(new Vector2d(50, -58))
                        //  .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(90))
                        //pause?
                        .build());

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
}
