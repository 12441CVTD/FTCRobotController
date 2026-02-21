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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

@TeleOp

public class OG_Decode_TeleOp2_BLUE extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DecodeArms2 launcher = null;
    //DecodeIntake intake = null;
    DecodeMecanumDrive2 chassis = null;
    LLMech limelight = null;
    BlueLimelightAutoAim vision;

    private Timer timer = new Timer();
    private final int delay = 1000;

    // booleans and stuff for control improvements
    boolean intakeIsOn = false;
    boolean intakeReverse = false;
    boolean amplification = false;
    boolean amplificationMAX = false;
    boolean gOpen = false;
    boolean wOpen = false;
    boolean transfersOn = false;
    boolean transfersReverse = false;
    boolean targetLock = false;

    private Gamepad previousGP2 = new Gamepad();
    private Gamepad currentGP2 = new Gamepad();



    @Override
    public void init() {
        launcher = new DecodeArms2(hardwareMap);
        //intake = new DecodeIntake(hardwareMap);
        chassis = new DecodeMecanumDrive2(hardwareMap);
        limelight = new LLMech(hardwareMap);
        limelight.getLlResult();
        launcher.turretCenter();
        launcher.gateClose();
        launcher.flipperUp();
        vision = new BlueLimelightAutoAim(hardwareMap);
    }

    @Override
    public void start(){
        limelight.startLL();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Initialized");
        vision.update();

        limelight.getLlResult();
        if (gamepad1.a){
            if (limelight.getLlResult() != null){ //checks to see if camera is seeing sonething that it is supposed to see
                targetLock = true;
                float botCorr = limelight.botCorrection();
                chassis.drive(-gamepad1.left_stick_x, gamepad1.left_stick_y, botCorr*.5);
            }
        }else {
            targetLock = false;
            chassis.drive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        }

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

        //to adjust speed with distance
        if (vision.hasTarget()) {

            double distance = vision.getDistanceMeters();

            double speed;

            if (distance < 1.5) {
                launcher.limelightLaunch(1100);
                speed = 0.55; // close
            } else if (distance < 2.5) {
                launcher.limelightLaunch(1300);
                speed = 0.65; // medium
            } else {
                launcher.limelightLaunch(1500);
                speed = 0.75; // far
            }
        }

        //transfers on
        if (currentGP2.a && !(previousGP2.a)) {
            if(transfersOn) {
                launcher.transferOff();
                transfersOn = false;
                transfersReverse = false;
            } else {
                launcher.transferOn();
                transfersOn = true;
                intakeReverse = false;
            }
        }

        //Reverse transfers
        if (currentGP2.b && !(previousGP2.b)) {
            if(transfersReverse) {
                launcher.transferOff();
                intakeReverse = false;
                intakeIsOn = false;
            } else {
                launcher.transferReverse();
                intakeIsOn = false;
                intakeReverse = true;
            }
        }


        if (currentGP2.y && !(previousGP2.y)) {
            if (!wOpen) {
                wOpen = true;
                launcher.gateOpen();
            } else {
                wOpen = false;
                launcher.gateClose();
            }
        }

        if (currentGP2.x && !(previousGP2.x)) {
            if (!gOpen) {
                gOpen = true;
                launcher.flipperUp();
            } else {
                gOpen = false;
                launcher.flipperDown();
            }
        }

        if(currentGP2.left_bumper && !(previousGP2.left_bumper)){
            if(amplificationMAX){
                launcher.powReversal();
                amplificationMAX = false;
                timer.schedule((TimerTask) new gateClose(), 0);
                gOpen = false;
                wOpen = false;
                transfersOn = false;
                transfersReverse = false;
                timer.schedule((TimerTask) new transferOff(), 0);
                timer.schedule((TimerTask) new flipperDown(), 0);
            } else {
                launcher.powAmpMAX();
                amplificationMAX = true;
                amplification = false;
                timer.schedule((TimerTask) new gateOpen(), 0);
                gOpen = true;
                wOpen = true;
                transfersOn = true;

                transfersReverse = false;
                timer.schedule((TimerTask) new transferOn(), 1500);
                timer.schedule((TimerTask) new flipperUp(), 2500);
            }
        }

        if(currentGP2.right_bumper && !(previousGP2.right_bumper)){
            if(amplification){
                launcher.powReversal();
                amplification = false;
                timer.schedule((TimerTask) new gateClose(), 0);
                gOpen = false;
                wOpen = false;
                transfersOn = false;
                transfersReverse = false;
                timer.schedule((TimerTask) new transferOff(), 0);
                timer.schedule((TimerTask) new flipperUp(), 0);
            } else {
                launcher.powAmp();
                timer.schedule((TimerTask) new gateOpen(), 0);
                gOpen = true;
                wOpen = true;
                transfersOn = true;
                transfersReverse = false;
                timer.schedule((TimerTask) new transferOn(), 1500);
                timer.schedule((TimerTask) new flipperDown(), 2500);


                amplification = true;
                amplificationMAX = false;
            }
        }

        if (currentGP2.x && !(previousGP2.x)) {
            if(transfersOn) {
                //launcher.transferFOff();
                //launcher.transferSOff();
                transfersOn = false;
                transfersReverse = false;
            } else {
                //launcher.transferFOn();
                //launcher.transferSOn();
                transfersOn = true;
                transfersReverse = false;
            }
        }

        if (currentGP2.y && !(previousGP2.y)) {
            if(transfersReverse) {
                //launcher.transferFOff();
                //launcher.transferSOff();
                transfersOn = false;
                transfersReverse = false;
            } else {
                //launcher.transferFR();
                //launcher.transferSR();
                transfersOn = false;
                transfersReverse = true;
            }
        }

        if (currentGP2.dpad_left) {
            launcher.turretLeft();
        }

        if (currentGP2.dpad_right) {
            launcher.turretRight();
        }

        if (currentGP2.dpad_up){
            launcher.turretCenter();
        }

        if (currentGP2.dpad_down){
            launcher.turretWallAim();
        }

        if(currentGP2.left_stick_x > 0){
            launcher.manualLeft();
        }

        if(currentGP2.left_stick_x < 0){
            launcher.manualRight();
        }

        if(vision.hasTarget()){
            double tx = vision.getTx();
            double kP = 0.0005; // safe starting value
            targetLock = true;
            if(Math.abs(tx) > 0.5) { // deadband
                launcher.adjustTurret(-kP * tx);

            }
        }

        telemetry.addData("Left turret position", launcher.getLeftPosition());
        telemetry.addData("Right turret position", launcher.getRightPosition());
        telemetry.addLine();
        telemetry.addData("Target Lock Activated:", targetLock);
        limelight.updateLLTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addData("amplification: ", amplification);
        telemetry.addData("amplificationMAX: ", amplificationMAX);
        telemetry.addData("flipper up: ", gOpen);
        telemetry.addData("gate open", wOpen);
        telemetry.addData("Upper launcher velocity", launcher.getUpVelocity());
        telemetry.addData("Lower launcher velocity", launcher.getDownVelocity());
        telemetry.update();

        /*
        if (currentGP2.right_bumper && !(previousGP2.right_bumper)) {
                launcher.powAmplification(powSet);
                if(powSet < 2){
                    powSet++;
                }else if(powSet == 2){
                    powSet = 0;
                }
        }

        if (currentGP2.left_bumper && !(previousGP2.left_bumper)) {
            launcher.powReversal(powSet);
            if(powSet > 0){
                powSet--;
            }else if(powSet == 0){
                powSet = 2;
            }
        }

         */

        /*if (currentGP2.y && !(previousGP2.y)) {
            if(!launcherIsOn) {
                launcher.on();
                launcherIsOn = true;
            } else {
                launcher.off();
                launcherIsOn = false;
            }
        } */





            /*if(gamepad2.dpad_up){
                armPow = 0.8;
            }
            if(gamepad2.dpad_down){
                armPow = -0.7;
            }*/


            //Slow Mode

            /*if(gamepad1.dpad_up){
                chassis.fourDrive(0.25,0.25,0.25,0.25);
            }
            if(gamepad1.dpad_down){
                chassis.fourDrive(-0.25,-0.25,-0.25,-0.25);
            }
            if(gamepad1.dpad_left){
                chassis.fourDrive(-0.25,0.25,0.25,-0.25);
            }
            if(gamepad1.dpad_right){
                chassis.fourDrive(0.25,-0.25,-0.25,0.25);
            }*/


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
//            // Show the elapsed game time and wheel power
//
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f), arm(%.2f)", fLPower, fRPower, armPow);
//            telemetry.addData("Servos", "lElbow (%.2f), rElbow (%.2f), wrist (%.2f), claw (%.2f)", lElbow.getPosition(), rElbow.getPosition(), wrist.getPosition(), claw.getPosition());
//            telemetry.addData("Positions", "IsOpened (%.2f), right (%.2f), arm(%.2f)", fLPower, fRPower, armPow);
//            telemetry.update();



    }

    public class transferOn extends TimerTask {

        @Override
        public void run() {
            launcher.transferOn();
        }
    }

    public class transferOff extends TimerTask {

        @Override
        public void run() {
            launcher.transferOff();
        }
    }

    public class gateOpen extends TimerTask {
        public void run() {
            launcher.gateOpen();
        }
    }

    public class gateClose extends TimerTask {

        @Override
        public void run() {
            launcher.gateClose();
        }
    }

    public class flipperUp extends TimerTask {

        @Override
        public void run() {
            launcher.flipperUp();
        }
    }
        public class flipperDown extends TimerTask {

            @Override
            public void run() {
                launcher.flipperDown();
            }
        }

}





