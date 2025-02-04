package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;


@Autonomous(name="Left_Path1", group="IntoTheDeep")

public class IntoTheDeep_Left01_12441 extends LinearOpMode {

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

    private int DELAY_BETWEEN_MOVES = 100;

    private ArrayList times = new ArrayList<String>();
    private double[] ElbowPositions = AutoConstants.ElbowConstants;



    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();

    public void runOpMode() {

        Pose2d beginPose = new Pose2d(-20, -60, Math.toRadians(90));

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

        // 1st Sample Drop (0 is open, 0.45 is close)

        //Position 5000 is too high

        timer.schedule(new lift(3000, 1), 0);
        timer.schedule(new elbow(1), 100);
        timer.schedule(new wrist(0.17), 100);
        timer.schedule(new claw(0), 2000);


        // 2nd Sample Pickup
        timer.schedule(new lift(20, 1), 3500);
        timer.schedule(new elbow(0), 3900);
        timer.schedule(new wrist(0.51), 3900);
        timer.schedule(new claw(0.45), 4000);
/*
        // 2nd Sample Drop
        timer.schedule(new lift(3000, 1), 3900);
        timer.schedule(new claw(0), 5500);

        // 3rd Sample Pickup
        timer.schedule(new lift(20, 1), 6000);
        timer.schedule(new claw(0.45), 7200);

        // 3rd Sample Drop
        timer.schedule(new lift(3000, 1), 7500);
        timer.schedule(new claw(0), 9000);

        // 4th Sample Pickup
        timer.schedule(new lift(20, 1), 9500);
        timer.schedule(new claw(0.45), 10800);

        // 4th Sample Drop
        timer.schedule(new lift(3000, 1), 11000);
        timer.schedule(new claw(0), 12600);
*/
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(-53, -52), Math.toRadians(225)) // Place
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-48, -38), Math.toRadians(90))
                        .waitSeconds(1) /*
                        .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-57, -38), Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-56, -26), Math.toRadians(180))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(225)) */
                        .build());



        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    class elbow extends TimerTask{
        private int positionNum;
        private double position;

        public elbow(int positionNum){
            this.positionNum = positionNum;
            position = -1;
        }

        private elbow(double position){
            this.position = position;
            positionNum = -1;
        }


        public void run(){

            if((positionNum == 0) && (lElbow.getPosition() == ElbowPositions[3])){

                double big = lElbow.getPosition();

                for(double i = 5-1; i >= 0; i--){
                    double move = Math.max(big * (i/5), 0);
                    timer.schedule(new elbow(move), ((long)250*(5-((int)i))));
                    if(move == 0){
                        i = -1;
                    }
                }
            }
            else if(positionNum > 0){
                lElbow.setPosition(ElbowPositions[positionNum]);
                rElbow.setPosition(ElbowPositions[positionNum]);
            }

            if(positionNum == -1){
                lElbow.setPosition(position);
                rElbow.setPosition(position);
            }



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
}
