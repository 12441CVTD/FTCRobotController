package org.firstinspires.ftc.teamcode;

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


@Autonomous(name="Right_Path1", group="IntoTheDeep")

public class IntoTheDeep_Red01_12441 extends LinearOpMode {

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

    public void runOpMode() {

        Pose2d beginPose = new Pose2d(20, -60, Math.toRadians(90));

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
        claw.setPosition(0.45);


        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //Specimen Place code NOTE: the robot must have a mid-low grip on the specimen to place properly;
        timer.schedule(new lift(3050, 1), 0);
        timer.schedule(new lift(1420, 1), 1725);
        timer.schedule(new lift(200, 1), 2100);
        timer.schedule(new claw(0), 2420);


        // Sample pickUP
        timer.schedule(new lift(20, 1), 4740);
        timer.schedule(new claw(0.45), 5680); //+100 from last
        timer.schedule(new lift(375, 1), 5880); //+240 from last

        // Sample drop
        timer.schedule(new claw(0), 7250);

        //Specimen pickUp
        timer.schedule(new lift(1130, 1), 11350); //10950
        timer.schedule(new claw(0.45), 14500); //11700
        timer.schedule(new lift(1160, 1), 14600);

        //Specimen place2
        timer.schedule(new lift(3050, 1), 17600);
        timer.schedule(new lift(1420, 1), 19950);
        timer.schedule(new lift(200, 1), 22800);
        timer.schedule(new claw(0), 20300);

        //end
        timer.schedule(new lift(0, 1), 23000);

        //Specimen pickup


        //Final Sample
        //timer.schedule(new lift(20, 1), 19100);

        //timer.schedule(new claw(0.45), 19150);
        //timer.schedule(new lift(1180, 1), 19290);
        //timer.schedule(new claw(0), 21200);

        //Specimen Rush
        //timer.schedule(new claw(0.45), 21700);
        //timer.schedule(new lift(3100, 1), 21950);
        //timer.schedule(new lift(1420, 1), 25500);
        //timer.schedule(new lift(200, 1), 25850);
        //timer.schedule(new claw(0), 26950);


        // Move arm into end position
        timer.schedule(new lift(0, 1), 30000);



        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(0, -31.7))
                        .waitSeconds(0.4)
                        .strafeTo(new Vector2d(0, -34.3))
                        .strafeTo(new Vector2d(50.65, -38))
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
                        .strafeTo(new Vector2d(3, -31.3))
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

//TimerTasks
    class elbow extends TimerTask{
        private double position;

        public elbow(double position){
            this.position = position;
        }

        public void run(){
            lElbow.setPosition(position);
            rElbow.setPosition(position);
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

    class claw extends TimerTask{
        private double position;

        public claw(double position){
            this.position = position;
        }

        public void run(){
            claw.setPosition(position);
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

    abstract class alift implements Action {
        int pos;
        double pow;
        int delay = 0;
        public alift(int pos) {
            this.pos = pos;
        }
        public alift(int pos, int delay) {
            this.pos = pos;
            this.delay = delay;
        }
        public alift(int pos, int delay, double pow){
            this.pos = pos;
            this.delay = delay;
            this.pow = pow;
        }

        public void run(){
            timer.schedule(new lift(pos, 1), delay);
        }

    }


}
