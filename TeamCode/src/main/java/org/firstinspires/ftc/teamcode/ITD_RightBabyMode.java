package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="RightBabyMode")

public class ITD_RightBabyMode extends LinearOpMode {

    private DcMotor fR;
    private DcMotor fL;
    private DcMotor bL;
    private DcMotor bR;

    @Override

    public void runOpMode()
    {


        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");


        bR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

//        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//we don't need this rn lol

        waitForStart();
        driveWay(0.3,0.3,0.3,0.3,500);
        sleep(1000);
        driveWay(0.5,-0.5,-0.5,0.5,1300);
        telemetry.addData("hi", ")");
        telemetry.update();
    }

    public void driveWay (double fLp, double bLp, double fRp, double bRp, long time){

       fL.setPower(fLp);
       bL.setPower(bLp);
       bR.setPower(bRp);
       fR.setPower(fRp);

        sleep(time);

        fL.setPower(0.0);
        bL.setPower(0.0);
        bR.setPower(0.0);
        fR.setPower(0.0);




    }


}

