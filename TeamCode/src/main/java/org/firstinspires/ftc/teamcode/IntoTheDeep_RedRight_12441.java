package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;

@Autonomous(name="RedRight", group="IntoTheDeep")

public class IntoTheDeep_RedRight_12441 extends LinearOpMode {

    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;
    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();

    public void runOpMode() {

        // Initialize the drive system variables.
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        timer.schedule(new shmove(0.3, 1600), 0);
        timer.schedule(new frontToBack(0.5, 2500), 1600);

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    //AutoLeft/Right
    class shmove extends TimerTask{
        private double power;
        private double time;


        public shmove(double power, double time){
            this.power = power;
            this.time = time;
            runtime.reset();
        }

        public void run(){
            while(opModeIsActive() && runtime.milliseconds() < time) {
                fR.setPower(power);
                fL.setPower(-power);
                bR.setPower(-power);
                bL.setPower(power);
            }
        }
    }

    //AutoBack/Forth
    class frontToBack extends TimerTask{
        private double power;
        private double time;

        public frontToBack(double power, double time){
            this.power = power;
            this.time = time;
            runtime.reset();
        }

        public void run(){
            while(opModeIsActive() && runtime.milliseconds() < time){
                fR.setPower(power);
                fL.setPower(power);
                bR.setPower(power);
                bL.setPower(power);
            }
        }
    }
    //AutoRotation
    class turn extends TimerTask{
        private double power;
        private double time;

        public turn(double power, double time){
            this.power = power;
            this.time = time;
            runtime.reset();
        }

        public void run(){
            while(opModeIsActive() && runtime.milliseconds() < time){
                fR.setPower(power);
                fL.setPower(-power);
                bR.setPower(power);
                bL.setPower(-power);
            }
        }

    }



}
