//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import java.util.Timer;
//import java.util.TimerTask;
//
//@Autonomous(name="12441_ITD_REDleft", group="Robot")
//public class IntoTheDeep_RedLeft_12441 extends LinearOpMode{
//
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor fL = null;
//    private DcMotor bL = null;
//    private DcMotor fR = null;
//    private DcMotor bR = null;
//    private Timer timer = new Timer();
//
//
//    @Override
//    public void runOpMode() {
////initilizae the drive sytem varibles
//        fL  = hardwareMap.get(DcMotor.class, "fL");
//        fR  = hardwareMap.get(DcMotor.class, "fR");
//        bL  = hardwareMap.get(DcMotor.class, "bL");
//        bR  = hardwareMap.get(DcMotor.class, "bR");
//
////ot drive fowad
//        fL.setDirection(DcMotor.Direction.FORWARD);
//        fR.setDirection(DcMotor.Direction.REVERSE);
//        bL.setDirection(DcMotor.Direction.FORWARD);
//        bR.setDirection(DcMotor.Direction.REVERSE);
//        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        waitforStart();
//    }
//    public void driveWay(double lF, double rF, double lB, double rB, double s){
//        runtime.reset();
//        while(opModeIsActive() && (runtime.milliseconds() < s)){
//            fL.setPower(lF);
//            fR.setPower(rF);
//            bL.setPower(lB);
//            bR.setPower(rB);
//        }
//        fL.setPower(0);
//        fR.setPower(0);
//        bL.setPower(0);
//        bR.setPower(0);
//    }
//
//}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Red_Left")

public class IntoTheDeep_RedLeft_12441 extends LinearOpMode {


    public DcMotor motorLeft;
    public DcMotor motorRight;



    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.8 ;
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override


    public void runOpMode() {

        motorLeft = hardwareMap.dcMotor.get ("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//reset encoders during initialization
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorLeft.getCurrentPosition(),
                motorRight.getCurrentPosition());
        telemetry.update();

        waitForStart();

//code motion here

        encoderDrive(0.5,  12,  12, 5.0);

    }




    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));



            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                telemetry.update();
            }



            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Display complete for the driver.
            telemetry.addData("Path", "Complete");
            telemetry.update();


            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }


    }


}



