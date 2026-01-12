package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

@Autonomous
public class DecodeAutoCloseBlue extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DecodeArms launcher = null;
    DecodeIntake intake = null;
    DecodeMecanumDrive chassis = null;

    Timer timer = new Timer();

    @Override
    public void runOpMode() throws InterruptedException {

        launcher = new DecodeArms(hardwareMap);
        intake = new DecodeIntake(hardwareMap);
        chassis = new DecodeMecanumDrive(hardwareMap);

        runtime = new ElapsedTime();
        timer = new Timer();

        waitForStart();

        runtime.reset();

        while(opModeIsActive() && runtime.milliseconds() < 2000){
            chassis.drive(0,0.4,0);
        }

        chassis.drive(0,0,0);

        runtime.reset();

        while(opModeIsActive() && runtime.milliseconds() < 500){
            launcher.powAmp();
            intake.on();
        }

        runtime.reset();

        while(opModeIsActive() && runtime.milliseconds() < 7000){
            launcher.gateOpen();
            //timer.schedule(new DecodeAutoCloseBlue.highGateOpen(), 0);
            //timer.schedule(new DecodeAutoCloseBlue.highGateClosed(), 5000);
        }

        runtime.reset();

        while(opModeIsActive() && runtime.milliseconds() < 1300){
            chassis.drive(0.5, 0, 0);
        }

        chassis.drive(0,0,0);
        launcher.gateClose(0.99);
        launcher.powReversal();
        intake.off();

        sleep(300);
    }
    /*class highGateOpen extends TimerTask {

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
