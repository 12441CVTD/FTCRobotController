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

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import java.util.ArrayList;
import java.util.Timer;


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

@TeleOp(name="Posfind", group="Testing")

public class AutoPositionFinder extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Gamepad previousGP2 = new Gamepad();
    private Gamepad currentGP2 = new Gamepad();

    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor fR = null;
    private DcMotor bR = null;

    private Servo lElbow = null;
    private Servo rElbow = null;

    private Servo claw = null;
    private Servo wrist = null;

    private ArrayList positions = new ArrayList<Pose2d>();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Pose2d beginPose = new Pose2d(20, -60, Math.toRadians(90));

        // Initialize the drive system variables.
        lElbow = hardwareMap.get(Servo.class, "lElbow");
        rElbow = hardwareMap.get(Servo.class, "rElbow");

        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        lElbow.setDirection(Servo.Direction.FORWARD);
        rElbow.setDirection(Servo.Direction.REVERSE);

        claw.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        lElbow.setPosition(0);
        rElbow.setPosition(0);
        wrist.setPosition(0.5);
        claw.setPosition(0.45);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            previousGP2.copy(currentGP2);
            currentGP2.copy(gamepad2);

            drive.updatePoseEstimate();
            
            if(currentGP2.a && !previousGP2.a){
                positions.add(drive.pose);
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Current_Position","(" + drive.pose.position.x + ", " + drive.pose.position.y + ") Heading: "
                               + (drive.pose.heading.toDouble()/(Math.PI/180)));

            for(int i = 0; i < positions.size(); i++){
                Pose2d snap = (Pose2d) positions.get(i);
                telemetry.addData("Snap_Position" + (i+1), "(" + snap.position.x + ", " + snap.position.y + ") Heading: "
                               + Math.toDegrees(snap.heading.toDouble()));
            }

            telemetry.update();
        }

        // public void
    }
}
