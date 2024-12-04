package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.TankDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;

@Autonomous(name="RedRight", group="IntoTheDeep")

public class IntoTheDeep_RedRight_12441 extends LinearOpMode {

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

        lArm = hardwareMap.get(DcMotor.class, "lArm");
        rArm = hardwareMap.get(DcMotor.class, "rArm");

        lElbow = hardwareMap.get(Servo.class, "lElbow");
        rElbow = hardwareMap.get(Servo.class, "rElbow");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

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

        lArm.setDirection(DcMotor.Direction.REVERSE);
        rArm.setDirection(DcMotor.Direction.FORWARD);

        lElbow.setDirection(Servo.Direction.REVERSE);
        rElbow.setDirection(Servo.Direction.FORWARD);

        claw.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        lElbow.setPosition(0.1);
        rElbow.setPosition(0.1);
        wrist.setPosition(0);
        claw.setPosition(0.4);

        sleep(50);


        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        timer.schedule(new lift(1000, 0.55), 500);
        timer.schedule(new lift(750, -0.5), 1700);


        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(0, -35))
                        .build());



        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

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
        private double time;
        private double power;

        public lift(double time, double power){
            this.time = time;
            this.power = power;
        }

        public void run(){
            runtime.reset();
            while(opModeIsActive() && (runtime.milliseconds() < time)){
                lArm.setPower(power);
                rArm.setPower(power);
            }
            lArm.setPower(0.04);
            rArm.setPower(0.04);
        }
    }




    //AutoLeft/Right
    class Spline extends TimerTask{

        public void run(){

        }
    }

    //AutoBack/Forth
    class strafe extends TimerTask{

        public void run(){

        }
    }
    //AutoRotation
    class turn extends TimerTask{


        public void run(){

        }
    }

}
