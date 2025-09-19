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

package org.firstinspires.ftc.teamcode.Decode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Timer;
import java.util.TimerTask;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.constants.TeleopConstants;

import org.firstinspires.ftc.teamcode.Decode.DecodeArms;
import org.firstinspires.ftc.teamcode.Decode.DecodeIntake;
import org.firstinspires.ftc.teamcode.Decode.DecodeMecanumDrive;




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

@TeleOp(name="Decode TeleOp THIS ONE", group="Linear OpMode")

public class Decode_TeleOp extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DecodeArms launcher = new DecodeArms();
    DecodeIntake intake = new DecodeIntake();
    DecodeMecanumDrive chassis = new DecodeMecanumDrive();

    private Timer timer = new Timer();

    // booleans and stuff for control improvements
    boolean isOn = false;

    private Gamepad previousGP2 = new Gamepad();
    private Gamepad currentGP2 = new Gamepad();

    @Override
    public void init() {
        launcher.init(hardwareMap);
        intake.init(hardwareMap);
        chassis.init(hardwareMap);
    }



    @Override
    public void loop() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        chassis.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).



        // act = hardwareMap.get(DcMotor.class, "actuator");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips




        // Wait for the game to start (driver presses START)



        // run until the end of the match (driver presses STOP)

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

        if (currentGP2.a && !(previousGP2.a)) {
            if(!isOn) {
                intake.on();
                isOn = true;
            } else {
                intake.off();
                isOn = false;
            }
        }



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




            /* if(gamepad1.a){
                act.setPower(-0.5);
            }
            if(gamepad1.y){
                act.setPower(0.5);
            }
            */

//            // Send calculated power to wheels
//            fL.setPower(fLPower);
//            fR.setPower(fRPower);
//            bL.setPower(bLPower);
//            bR.setPower(bRPower);
//
//
//
//            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f), arm(%.2f)", fLPower, fRPower, armPow);
//            telemetry.addData("Servos", "lElbow (%.2f), rElbow (%.2f), wrist (%.2f), claw (%.2f)", lElbow.getPosition(), rElbow.getPosition(), wrist.getPosition(), claw.getPosition());
//            telemetry.addData("Positions", "IsOpened (%.2f), right (%.2f), arm(%.2f)", fLPower, fRPower, armPow);
//            telemetry.update();



    }



}





