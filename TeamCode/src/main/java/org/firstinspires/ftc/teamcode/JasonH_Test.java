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
    private Servo claw = null;
    private Timer timer = new Timer();

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

        claw = hardwareMap.get(Servo.class, "claw");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        claw.setDirection(Servo.Direction.FORWARD);
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double fLPower;
            double bLPower;
            double fRPower;
            double bRPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean a = gamepad1.a;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            fLPower    = Range.clip(drive + turn, -0.5, 0.5) ;
            fRPower   = Range.clip(drive - turn, -0.5, 0.5) ;
            bLPower    = Range.clip(drive + turn, -0.5, 0.5) ;
            bRPower   = Range.clip(drive - turn, -0.5, 0.5) ;
            if(gamepad1.left_bumper) {
                fLPower = -0.5;
                fRPower = 0.5;
                bLPower = 0.5;
                bRPower = -0.5;
            }
            if(gamepad1.right_bumper) {
                fLPower = 0.5;
                fRPower = -+0.5;
                bLPower = -0.5;
                bRPower = 0.5;
            }
            if(gamepad1.left_trigger > 0){
                claw.setPosition(0.6);
            }
            if(gamepad1.right_trigger > 0){
                claw.setPosition(0.2);
            }
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            if(x){
                timer.schedule(new shmove(.5, 1000), 0);
            }
            if(y){
                timer.schedule(new frontToBack(.5, 1000), 0);
            }
            if(a){
                timer.schedule(new turn(.5 , 1000), 0);
            }
            // Send calculated power to wheels
            fL.setPower(fLPower);
            fR.setPower(fRPower);
            bL.setPower(bLPower);
            bR.setPower(bRPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", fLPower, fRPower);
            telemetry.update();
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





