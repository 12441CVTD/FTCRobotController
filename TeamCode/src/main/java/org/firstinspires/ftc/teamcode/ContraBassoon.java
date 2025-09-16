///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
///*
// * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
// * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
// * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all linear OpModes contain.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
// */
//
//@TeleOp(name="usage 1, please use", group="Linear OpMode")
//public class ContraBassoon extends LinearOpMode {
//
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor fL = null;
//    private DcMotor fR = null;
//    private DcMotor bL = null;
//    private DcMotor bR = null;
//    private DcMotor zoom = null;
//    private Servo flipper = null;
//    private Servo turny = null;
//    private Servo grabby = null;*/
//    private Servo woosh = null;
//    @Override
//    public void runOpMode() {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        fL  = hardwareMap.get(DcMotor.class, "fL");
//        fR = hardwareMap.get(DcMotor.class, "fR");
//        bL  = hardwareMap.get(DcMotor.class, "bL");
//        bR = hardwareMap.get(DcMotor.class, "bR");
//        zoom = hardwareMap.get(DcMotor.class, "zoom");
//        flipper = hardwareMap.get(Servo.class, "flipper");
//        turny = hardwareMap.get(Servo.class, "turny");
//        grabby = hardwareMap.get(Servo.class, "grabby");
//        woosh = hardwareMap.get(Servo.class, "woosh");
//         To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//         Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
//         Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        fL.setDirection(DcMotor.Direction.REVERSE);
//        fR.setDirection(DcMotor.Direction.FORWARD);
//        bL.setDirection(DcMotor.Direction.REVERSE);
//        bR.setDirection(DcMotor.Direction.FORWARD);
//        zoom.setDirection(DcMotor.Direction.FORWARD);
//
//        double fLp;
//        double fRp;
//        double bLp;
//        double bRp;
//        double zoomP;
//
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//        runtime.reset();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            // Setup a variable for each drive wheel to save power level for telemetry
//
//            // Choose to drive using either Tank Mode, or POV Mode
//            // Comment out the method that's not used.  The default below is POV.
//
//            // POV Mode uses left stick to go forward, and right stick to turn.
//            // - This uses basic math to combine motions and is easier to drive straight.
//            double drive = -gamepad1.left_stick_y;
//            double turn  =  gamepad1.right_stick_x;
//            double strafe = 0;
//            double strafe = gamepad1.left_stick_x;
//            fLp = Range.clip(drive + turn + strafe, -1.0, 1.0);
//            fRp = Range.clip(drive - turn - strafe, -1.0, 1.0);
//            bLp = Range.clip(drive + turn - strafe, -1.0, 1.0);
//            bRp = Range.clip(drive - turn + strafe, -1.0, 1.0);
//            zoomP = -gamepad2.left_stick_y;
//
//            if(0.5 > gamepad1.right_trigger && 0.5 < gamepad1.left_trigger){
//              fLp = Range.clip(drive + turn + strafe + 1.0, -1.0, 1.0);
//              fLp = Range.clip(drive + turn - strafe - 1.0, -1.0, 1.0);
//              fLp = -0.85;
//              fRp = 0.85;
//              bLp = 0.85;
//              bRp = -0.85;
//            }
//            else if(0.5 < gamepad1.right_trigger && 0.5 > gamepad1.left_trigger){
//              fLp = 0.85;
//              fRp = -0.85;
//              bLp = -0.85;
//              bRp = 0.85;
//            }
//            if(gamepad1.dpad_up){
//                fLp = 0.3;
//                fRp = 0.3;
//                bLp = 0.3;
//                bRp = 0.3;
//            }
//            else if(gamepad1.dpad_down){
//                fLp = -0.3;
//                fRp = -0.3;
//                bLp = -0.3;
//                bRp = -0.3;
//            }
//            else if(gamepad1.dpad_right){
//                fLp = 0.3;
//                fRp = -0.3;
//                bLp = 0.3;
//                bRp = -0.3;
//            }
//            else if(gamepad1.dpad_left){
//                fLp = -0.3;
//                fRp = 0.3;
//                bLp = -0.3;
//                bRp = 0.3;
//            }
//            if (gamepad2.dpad_up){
//                zoomP= 0.3;
//            }
//            else if(gamepad2.dpad_down){
//                zoomP= -0.3;
//            }
//            if (gamepad2.x){
//                flipper.setPostion(1);
//            }
//            else if (gamepad2.y){
//                flipper.setPosition(0);
//            }
//            if(gamepad2.right_bumper){
//                runtime.reset();
//                if(opModeIsActive() && (runtime.milliseconds() == 1)){
//                grabby.setPosition(0);
//                }
//                if(opModeIsActive() && (runtime.milliseconds() == 50)){
//                flippy.setPosition(0);
//                }
//            }
//            if(gamepad2.left_bumper){
//                runtime.reset();
//                if(opModeIsActive() && (runtime.milliseconds() == 1)){
//                grabby.setPosition(1);
//                }
//                if(opModeIsActive() && (runtime.milliseconds() == 50)){
//                flippy.setPosition(1);
//                }
//            }*/
//            if(gamepad2.a){
//                woosh.setPosition(0.5);
//            }
//            // Tank Mode is cringe.
//            // - This requires no math, but it is hard to drive forward slowly and keep straight.
//             fLp  = -gamepad1.left_stick_y ;
//             fRp = -gamepad1.right_stick_y ;
//             bLp = -gamepad1.left_stick_y;
//             bRp = -gamepad1.right_stick_y;
//
//            // Send calculated power to wheels
//            fL.setPower(fLp);
//            fR.setPower(fRp);
//            bL.setPower(bLp);
//            bR.setPower(bRp);
//            zoom.setPower(zoomP);
//
//            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", fLp, fRp, bLp, bRp, zoomP);
//            telemetry.update();
//        }
//    }
//}
//
////Odometry code:
////(change stuff ben)
//
//package org.firstinspires.ftc.teamcode.odometry
//
//interface Odometry {
//    fun setAngleRad(angle_rad: Double)
//    fun addAngleBias(angle_rad: Double)
//}
//
////THREE WHEEL ODOMETRY TYPE BEAT (taken from some random place dont even worry about it) SHOWN BELOW:
//
//package org.firstinspires.ftc.teamcode.odometry
//
//import org.firstinspires.ftc.teamcode.field.*
//import org.firstinspires.ftc.teamcode.movement.*
//import org.firstinspires.ftc.teamcode.movement.basicDriveFunctions.*
//
//object ThreeWheel : Odometry{
//        var yTraveled=0.0
//        var xTraveled=0.0
//        var degTraveled=0.0
//
//// last encoder positions
//private var last_l_encoder=0
//private var last_r_encoder=0
//private var last_a_encoder=0
//
//        // used for reading angle absolutely not integrated
//        var angleRadBias=0.0
//
//private var lastRawAngle=0.0
//
//        fun update(curr_l_encoder:Int,curr_r_encoder:Int,curr_a_encoder:Int,leftInchesPerTick:Double,rightInchesPerTick:Double,auxInchesPerTick:Double,turnTrackWidth:Double,auxTrackWidth:Double){
//        DrivePosition.odometer=this
//
//        val lWheelDelta=(curr_l_encoder-last_l_encoder)*leftInchesPerTick
//        val rWheelDelta=(curr_r_encoder-last_r_encoder)*rightInchesPerTick
//        val aWheelDelta=(curr_a_encoder-last_a_encoder)*auxInchesPerTick
//
//
//        // calculate angle change for running arc integration and aux prediction
//        val angleIncrement=(lWheelDelta-rWheelDelta)/turnTrackWidth
//
//        // use absolute for actual angle
//        val leftTotal=curr_l_encoder*rightInchesPerTick
//        val rightTotal=curr_r_encoder*rightInchesPerTick
//        lastRawAngle=((leftTotal-rightTotal)/turnTrackWidth)
//        val finalAngleRad=lastRawAngle+angleRadBias
//
//        // the aux wheel moves when we rotate, so cancel this out with a prediction
//        val aux_prediction=angleIncrement*auxTrackWidth
//
//        val yDelta=(lWheelDelta+rWheelDelta)/2.0
//        val xDelta=aWheelDelta-aux_prediction
//
//        DrivePosition.updatePos(Pose(xDelta,yDelta,angleIncrement),Angle.createUnwrappedRad(finalAngleRad))
//
//        last_l_encoder=curr_l_encoder
//        last_r_encoder=curr_r_encoder
//        last_a_encoder=curr_a_encoder
//        }
//
//        override fun setAngleRad(angle_rad:Double){
//        angleRadBias=angle_rad-lastRawAngle
//        }
//
//        fun addAngleRad(angle_rad:Double){
//        angleRadBias+=angle_rad
//        }
//
//        override fun addAngleBias(angle_rad:Double){
//        angleRadBias+=angle_rad
//        }
//}
//*/
