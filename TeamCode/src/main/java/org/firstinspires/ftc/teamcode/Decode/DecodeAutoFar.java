package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

@Autonomous
public class DecodeAutoFar extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();

    DecodeArms launcher = null;
    DecodeIntake intake = null;
    DecodeMecanumDrive chassis = null;

    @Override
    public void runOpMode() throws InterruptedException {

        launcher = new DecodeArms(hardwareMap);
        intake = new DecodeIntake(hardwareMap);
        chassis = new DecodeMecanumDrive(hardwareMap);

        timer = new Timer();
        runtime = new ElapsedTime();

        waitForStart();

        //chassis.drive(0,0.2,0);
        intake.on();

        runtime.reset();

        while(opModeIsActive() && runtime.milliseconds() < 1500){
            launcher.powAmpMAX();

        }
        runtime.reset();

        while(opModeIsActive() && runtime.milliseconds() < 8000){
            launcher.gateOpen();
            //timer.schedule(new highGateOpen(), 0);
            //timer.schedule(new highGateClosed(), 5000);
        }

        runtime.reset();
        while(opModeIsActive() && runtime.milliseconds() < 400) {
            chassis.drive(0,-0.5,0);
        }
        launcher.powReversal();
        launcher.gateClose();


        sleep(300);

    }

    /*class highGateOpen extends TimerTask{

        @Override
        public void run() {
            launcher.highGateOpen();
        }
    }

    class highGateClosed extends TimerTask{

        @Override
        public void run() {
            launcher.highGateClosed();
        }
    }*/
}

