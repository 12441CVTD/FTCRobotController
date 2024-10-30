/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Timer;
import java.util.TimerTask;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Test01", group="Linear OpMode")

public class JasonH_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor fR = null;
    private DcMotor bR = null;
    private DcMotor lArm = null;
    private DcMotor rArm = null;

    private Servo lElbow = null;
    private Servo rElbow = null;
    private Servo claw = null;
    private Servo wrist = null;



    private Timer timer = new Timer();

    // booleans and stuff for control improvements
    private Gamepad previousGP2 = new Gamepad();
    private Gamepad currentGP2 = new Gamepad();

    boolean isOpened = false;
    boolean turney;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fL  = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL  = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        lArm = hardwareMap.get(DcMotor.class, "lArm");
        rArm = hardwareMap.get(DcMotor.class, "rArm");

        lElbow = hardwareMap.get(Servo.class, "lElbow");
        rElbow = hardwareMap.get(Servo.class, "rElbow");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        lArm.setDirection(DcMotor.Direction.REVERSE);
        rArm.setDirection(DcMotor.Direction.FORWARD);

        lElbow.setDirection(Servo.Direction.REVERSE);
        rElbow.setDirection(Servo.Direction.FORWARD);

        claw.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);

        //start position
        lElbow.setPosition(0);
        rElbow.setPosition(0);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Can be used to make toggleable inputs
            previousGP2.copy(currentGP2);
            currentGP2.copy(gamepad2);
            // Format: if(currentGP2.*insertInput* && !previousGP2.*insertInput*){`

            // Setup a variable for each drive wheel to save power level for telemetry
            double fLPower;
            double bLPower;
            double fRPower;
            double bRPower;

            double deceleration = 0.0;


            double armPow = 0.01;




            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
         //   boolean x = gamepad1.x;
         //   boolean y = gamepad1.y;
         //   boolean a = gamepad1.a;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            if(gamepad1.right_bumper){
                deceleration = 0.3;
            }

            fLPower    = Range.clip(drive + turn, -0.5 + deceleration, 0.5 - deceleration) ;
            fRPower   = Range.clip(drive - turn, -0.5 + deceleration, 0.5 - deceleration) ;
            bLPower    = Range.clip(drive + turn, -0.5 + deceleration, 0.5 - deceleration) ;
            bRPower   = Range.clip(drive - turn, -0.5 + deceleration, 0.5 - deceleration) ;

            if(gamepad2.dpad_up){
                armPow = 0.5;
            }
            if(gamepad2.dpad_down){
                armPow = -0.5;
            }

            if(gamepad1.left_trigger > 0) {
                fLPower = -0.5;
                fRPower = 0.5;
                bLPower = 0.5;
                bRPower = -0.5;
            }
            if(gamepad1.right_trigger > 0) {
                fLPower = 0.5;
                fRPower = -+0.5;
                bLPower = -0.5;
                bRPower = 0.5;
            }
            //Stuff has changed
            //0 == ground
            //1 == danger

            //mid
            if(gamepad2.left_stick_button){
                lElbow.setPosition(0);
                rElbow.setPosition(0);
            }
            if(gamepad2.left_trigger > 0){
                lElbow.setPosition(0.25);
                rElbow.setPosition(0.25);
            }

            if(gamepad2.right_trigger > 0){
                lElbow.setPosition(0.12);
                rElbow.setPosition(0.12);
            }

            if(gamepad2.left_bumper){
                lElbow.setPosition(0.4);
                rElbow.setPosition(0.4);
            }

            if(gamepad2.right_bumper){
                lElbow.setPosition(0.03);
                rElbow.setPosition(0.03);
            }

            if(currentGP2.a && !previousGP2.a){
                isOpened = !isOpened;
            }
            if(gamepad2.b){
                wrist.setPosition(0.035);
            }
            if(gamepad2.y){
                wrist.setPosition(0.1);
            }
            if(gamepad2.right_stick_button){
                wrist.setPosition(0.05);
            }

            // Send calculated power to wheels
            fL.setPower(fLPower);
            fR.setPower(fRPower);
            bL.setPower(bLPower);
            bR.setPower(bRPower);
            // Send power to the arms
            lArm.setPower(armPow);
            rArm.setPower(armPow);
            // Check claw position
            if(isOpened){
                claw.setPosition(0.3);
            }
            else if(!isOpened){
                claw.setPosition(0.7);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), arm(%.2f)", fLPower, fRPower, armPow);
            telemetry.addData("Servos", "lElbow (%.2f), rElbow (%.2f)", lElbow.getPosition(), rElbow.getPosition());
            telemetry.update();
        }

       // public void
    }

    class elbowShmove extends TimerTask{
        private double position;

        public elbowShmove(double position){
            this.position = position;
        }

        public void run(){
            lElbow.setPosition(position);
            rElbow.setPosition(position);
        }
    }

    class wristShmove extends TimerTask{
        private double position;

        public wristShmove(double position){
            this.position = position;
        }

        public void run(){
            wrist.setPosition(position);
        }
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





