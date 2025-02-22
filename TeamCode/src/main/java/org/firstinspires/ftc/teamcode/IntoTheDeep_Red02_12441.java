package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import java.util.Timer;
import java.util.TimerTask;


@Autonomous(name="Right_Path2", group="IntoTheDeep")

public class IntoTheDeep_Red02_12441 extends LinearOpMode {

    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;

    private DcMotor lArm = null;
    private DcMotor rArm = null;

    private Servo lElbow = null;
    private Servo rElbow = null;
    private Servo claw = null;
    private Servo wrist = null;

    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();

    private boolean around = false;

    private int[] LiftConstants = AutoConstants.SpecimenLiftConstants;
    private double[] ClawConstants = AutoConstants.ClawConstants;

    class claw extends TimerTask{
        private double position;

        public claw(double position){
            this.position = position;
        }

        public void run(){
            claw.setPosition(position);
        }

    }

    class wrist extends TimerTask{
        private double position;

        public wrist(double position){
            this.position = position;
        }

        public void run(){
            wrist.setPosition(position);
        }
    }

    class lift extends TimerTask{
        private int position;
        private double power;

        public lift(int position, double power){
            this.position = position;
            this.power = power;
        }

        public void run(){

            lArm.setTargetPosition(position);
            rArm.setTargetPosition(position);

            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lArm.setPower(power);
            rArm.setPower(power);

        }
    }

    class elbow extends TimerTask{
        double pos;

        public elbow(double position){
            pos = position;
        }

        public void run(){
            lElbow.setPosition(pos);
            rElbow.setPosition(pos);
        }


    }

    public void runOpMode() {

        // Initialize the drive system variables.
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        lElbow = hardwareMap.get(Servo.class, "lElbow");
        rElbow = hardwareMap.get(Servo.class, "rElbow");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // (arm) encoder stuff
        lArm = hardwareMap.dcMotor.get("lArm");
        rArm = hardwareMap.dcMotor.get("rArm");

        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lArm.setDirection(DcMotor.Direction.REVERSE);
        rArm.setDirection(DcMotor.Direction.FORWARD);


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

        lElbow.setDirection(Servo.Direction.FORWARD);
        rElbow.setDirection(Servo.Direction.REVERSE);

        claw.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);

        lElbow.setPosition(0);
        rElbow.setPosition(0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(20, -60, Math.PI /2));

        waitForStart();

        lArm.setTargetPosition(0);
        rArm.setTargetPosition(0);

        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lElbow.setPosition(0);
        rElbow.setPosition(0);
        wrist.setPosition(0.5);
        claw.setPosition(0.43);

        while(opModeIsActive()){

            GoGoGo(drive);

            sleep(15000);
            break;

        }


/*
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
*/
    }

    //Arm Actions
    public class Kickback implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!around){
                around = !around;
                timer.schedule(new elbow(0.405), 0);
                timer.schedule(new wrist(0.34), 0);
                timer.schedule(new elbow(0.55), 600);
                timer.schedule(new elbow(0.68), 1300);
            }
            else{
                around = !around;
                timer.schedule(new elbow(0.405), 0);
                timer.schedule(new wrist(0.5), 0);
                timer.schedule(new elbow(0.2025), 700);
                timer.schedule(new elbow(0), 1250);
            }
            return false;
        }

    }

    //Claw Actions

    public class OpenClaw implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(ClawConstants[0]);
            return false;
        }

    }

    public class CloseClaw implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(ClawConstants[1]);
            return false;
        }
    }

    //Lift Actions

    public class UpLift implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            lArm.setTargetPosition(LiftConstants[2]);
            rArm.setTargetPosition(LiftConstants[2]);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setPower(1);
            rArm.setPower(1);
            return false;
        }
    }

    public class toMid implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            lArm.setTargetPosition(LiftConstants[1]);
            rArm.setTargetPosition(LiftConstants[1]);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setPower(1);
            rArm.setPower(1);
            return false;
        }

    }

    public class DownShift implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            lArm.setTargetPosition(LiftConstants[0]);
            rArm.setTargetPosition(LiftConstants[0]);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setPower(1);
            rArm.setPower(1);
            return false;
        }

    }
    // Big Actions
    public class GrabFromGround implements Action{
        TrajectoryActionBuilder actionBuilder;

        public GrabFromGround(MecanumDrive drive){
            actionBuilder = drive.actionBuilder(drive.pose);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Actions.runBlocking(new SequentialAction(actionBuilder.afterTime(0, new CloseClaw()).build(),
                                                    actionBuilder.afterTime(0.2, new Kickback()).build(),
                                                    actionBuilder.afterTime(1.8, new OpenClaw()).build()));
            return false;
        }
    }

    public class SpecimenPlace implements Action{
        TrajectoryActionBuilder actionBuilder;
        double variance;
        public SpecimenPlace(MecanumDrive drive, double diff){
            actionBuilder = drive.actionBuilder(drive.pose);
            variance = diff;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Actions.runBlocking(actionBuilder.);
            return false;
        }
    }

    private void GoGoGo(MecanumDrive drive){
        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(drive.pose);

        Actions.runBlocking(new SequentialAction(new ParallelAction(actionBuilder.afterTime(0, new UpLift()).build(), actionBuilder.afterTime(0.05, actionBuilder.strafeTo(new Vector2d(0, -32)).build()).build()),
                                                    new ParallelAction(new toMid(), actionBuilder.afterTime(0.25, new OpenClaw()).build())));

        Actions.runBlocking(new SequentialAction(new ParallelAction(actionBuilder.strafeTo(new Vector2d(48, -36)).build(), actionBuilder.afterTime(0.85, new DownShift()).build()),
                                                    actionBuilder.afterTime(0.1, new GrabFromGround(drive)).build()));

        Actions.runBlocking(new SequentialAction(new ParallelAction(actionBuilder.strafeTo(new Vector2d(59, -36)).build(), actionBuilder.afterTime(0.1, new Kickback()).build()),
                                                    actionBuilder.afterTime(0, new GrabFromGround(drive)).build()));

        actionBuilder = drive.actionBuilder(drive.pose);

        Actions.runBlocking(new SequentialAction(actionBuilder.strafeTo(new Vector2d(60, -48)).build(),
                                                 actionBuilder.afterTime(0, new CloseClaw()).build()));

        Actions.runBlocking(new ParallelAction(actionBuilder.strafeTo(new Vector2d(5, -32)).build(),
                                                actionBuilder.afterTime(0, new Kickback()).build(),
                                                actionBuilder.afterTime(0, new UpLift()).build()));

    }

}
