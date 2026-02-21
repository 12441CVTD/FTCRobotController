package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

@Autonomous
public class V2AutoBasic extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();

    DecodeArms2 launcher = null;
    DecodeMecanumDrive2 chassis = null;

    @Override
    public void runOpMode() throws InterruptedException {

        launcher = new DecodeArms2(hardwareMap);
        chassis = new DecodeMecanumDrive2(hardwareMap);

        timer = new Timer();
        runtime = new ElapsedTime();

        waitForStart();

        runtime.reset();
        chassis.drive(0,-0.5,0);
        sleep(650);
        chassis.drive(0,0,0);
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

