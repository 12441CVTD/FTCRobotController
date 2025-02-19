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
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.constants.TeleopConstants;


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

@TeleOp(name="ITD TeleOp THIS ONE", group="Linear OpMode")

public class ITD_TeleOP extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor fR = null;
    private DcMotor bR = null;
    private DcMotor lArm = null;
    private DcMotor rArm = null;
    // private DcMotor act = null;
    private Servo lElbow = null;
    private Servo rElbow = null;
    private Servo claw = null;
    private Servo wrist = null;


    private Timer timer = new Timer();

    // booleans and stuff for control improvements
    private Gamepad previousGP2 = new Gamepad();
    private Gamepad currentGP2 = new Gamepad();

    boolean isOpened = true;
    boolean isDown = false;
    boolean holdUp = false;
    boolean around = false;

    private double[] elbowPositions = TeleopConstants.ElbowConstants;



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
        // act = hardwareMap.get(DcMotor.class, "actuator");

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

        lElbow.setDirection(Servo.Direction.FORWARD);
        rElbow.setDirection(Servo.Direction.REVERSE);

    //    act.setDirection(DcMotor.Direction.FORWARD);

        claw.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);

        //start position
        lElbow.setPosition(0);
        rElbow.setPosition(0);
        wrist.setPosition(0.51);
        claw.setPosition(0.4);


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

            double armPow = 0.04;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            fLPower    = Range.clip(drive + turn, -0.8, 0.8) ;
            fRPower   = Range.clip(drive - turn, -0.8, 0.8) ;
            bLPower    = Range.clip(drive + turn, -0.8, 0.8) ;
            bRPower   = Range.clip(drive - turn, -0.8, 0.8) ;

            if(gamepad2.dpad_up){
                armPow = 0.8;
            }
            if(gamepad2.dpad_down){
                armPow = -0.7;
            }

            if(gamepad1.left_trigger > 0) {
                fLPower = -0.8;
                fRPower = 0.8;
                bLPower = 0.8;
                bRPower = -0.8;
            }
            if(gamepad1.right_trigger > 0) {
                fLPower = 0.8;
                fRPower = -0.8;
                bLPower = -0.8;
                bRPower = 0.8;
            }
            //Slow Mode
            if(gamepad1.dpad_up){
                fLPower = 0.25;
                fRPower = 0.25;
                bLPower = 0.25;
                bRPower = 0.25;
            }
            if(gamepad1.dpad_down){
                fLPower = -0.25;
                fRPower = -0.25;
                bLPower = -0.25;
                bRPower = -0.25;
            }
            if(gamepad1.dpad_left){
                fLPower = -0.25;
                fRPower = 0.25;
                bLPower = 0.25;
                bRPower = -0.25;
            }
            if(gamepad1.dpad_right){
                fLPower = 0.25;
                fRPower = -0.25;
                bLPower = -0.25;
                bRPower = 0.25;
            }
            //Stuff has changed
            //0 == ground
            //1 == danger



            // Mid low
            if(gamepad2.left_trigger > 0){
                lElbow.setPosition(0.08);
                rElbow.setPosition(0.08);
            }
            // All around the world
            if(currentGP2.right_trigger > 0 && !(previousGP2.right_trigger > 0)){
                holdUp = true;
                theWORLD();
            }
            // Lowest
            if(gamepad2.left_bumper){
                splitMove(lElbow.getPosition(), 0.0, 5, 250);
                //lElbow.setPosition(0.0);
                //rElbow.setPosition(0.0);
            }
            // Highest
            if(gamepad2.right_bumper){
                lElbow.setPosition(0.31);
                rElbow.setPosition(0.31);
            }
                                                    // Checks for if the wrist is in one of the position
            if(currentGP2.a && !previousGP2.a && ((wrist.getPosition() != 0.05) || (wrist.getPosition() != 1))){
                isOpened = !isOpened;
            }
            if(currentGP2.y && !previousGP2.y){
                isDown = !isDown;
                holdUp = false;
                if(isOpened){
                    isOpened = false;
                }
            }
            if(currentGP2.x && !previousGP2.x){
                holdUp = !holdUp;
                wrist.setPosition(0.25);
            }
            /* if(gamepad1.a){
                act.setPower(-0.5);
            }
            if(gamepad1.y){
                act.setPower(0.5);
            }
            */

            // Send calculated power to wheels
            fL.setPower(fLPower);
            fR.setPower(fRPower);
            bL.setPower(bLPower);
            bR.setPower(bRPower);

            // Send power to the arms
            // Sends power to the arms
            lArm.setPower(armPow);
            rArm.setPower(armPow);

            // Check claw positions
            if(isOpened){
                claw.setPosition(0.01);
            }
            else if(!isOpened){
                claw.setPosition(0.4);
            }
            // 0.05 == THROWING
            if(isDown && !holdUp){
                wrist.setPosition(0.17);
            }
            else if(!isDown && !holdUp){
                wrist.setPosition(0.51);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), arm(%.2f)", fLPower, fRPower, armPow);
            telemetry.addData("Servos", "lElbow (%.2f), rElbow (%.2f), wrist (%.2f), claw (%.2f)", lElbow.getPosition(), rElbow.getPosition(), wrist.getPosition(), claw.getPosition());
            telemetry.addData("Positions", "IsOpened (%.2f), right (%.2f), arm(%.2f)", fLPower, fRPower, armPow);
            telemetry.update();
        }


    }

    public void splitMove(double InitPos, double FinalPos, int numSp, long times){

        /*
        Goal: Move a servo, probably the arm, from its current position to the goal position by splitting
        the movement into "x" different positions, including the finalPos.

        How:
            1. Take the initPos (Ex: 1)
            2. Take the finalPos (Ex: 0)                                      Between Positions
        3. Find positions between them according to "X"(Ex: "X" = 3 so 1 | 0.75, 0.5, 0.25 | 0)
         */

        double big = Math.max(FinalPos, InitPos);
        ArrayList<Double> valSave = new ArrayList<>();
        valSave.add(0, big);


        for(double i = numSp-1; i >= 0; i--){
            double move = Math.max(big * ((i/numSp)), Math.min(InitPos, FinalPos));
            valSave.add(move);
            if(move == Math.min(InitPos, FinalPos) || i == 0){
                i = -1;
                for(int x = 0; x < valSave.size(); x++){
                    if(big == InitPos){
                        timer.schedule(new elbowShmove(valSave.get(x)), (times*(x+1)));
                    }
                    else{
                        timer.schedule(new elbowShmove(valSave.get(valSave.size()-(x+1))), (times*(x+1)));
                    }
                }
            }
        }


    }

    class delaySplit extends TimerTask{
        private double initPos;
        private double finalPos;
        private int numPos;
        private long times;


        public delaySplit(double initPos, double finalPos, int numPos, long times){
            this.initPos = initPos;
            this.finalPos = finalPos;
            this.numPos = numPos;
            this.times = times;
        }


        public void run(){
            splitMove(initPos, finalPos, numPos, times);
        }

    }


    public void theWORLD(){
        if(!around){
//            lElbow.setPosition(0.405);
//            rElbow.setPosition(0.405);
            //timer.schedule(new delaySplit(lElbow.getPosition(), 0.68, 5, 250), 0);
            splitMove(lElbow.getPosition(), 0.68, 5, 400);
            //timer.schedule(new wristShmove(0.34), 500);
            wrist.setPosition(0.34);
            around = !around;
        }
        else{
            timer.schedule(new elbowShmove(0.405), 0);
            timer.schedule(new wristShmove(0.68), 25);
            timer.schedule(new delaySplit(0.405, 0.14, 4, 250), 600);
            around = !around;
        }
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





