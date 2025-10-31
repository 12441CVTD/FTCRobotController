package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class DecodeAutoClose extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DecodeArms launcher = null;
    DecodeIntake intake = null;
    DecodeMecanumDrive chassis = null;

    @Override
    public void runOpMode() throws InterruptedException {

        launcher = new DecodeArms(hardwareMap);
        intake = new DecodeIntake(hardwareMap);
        chassis = new DecodeMecanumDrive(hardwareMap);

        runtime = new ElapsedTime();

        waitForStart();

        runtime.reset();

        while(opModeIsActive() && runtime.milliseconds() < 2000){
            chassis.drive(0,0.4,0);
        }

        chassis.drive(0,0,0);

        runtime.reset();

        while(opModeIsActive() && runtime.milliseconds() < 500){
            launcher.powAmplification();
            intake.on();
        }

        runtime.reset();

        while(opModeIsActive() && runtime.milliseconds() < 7000){
            launcher.gateOpen();
        }

        runtime.reset();

        while(opModeIsActive() && runtime.milliseconds() < 1000){
            chassis.drive(0, 0.3, 0);
        }

        chassis.drive(0,0,0);
        launcher.gateClose();
        launcher.powReversal();
        intake.off();

        sleep(300);
    }
}
