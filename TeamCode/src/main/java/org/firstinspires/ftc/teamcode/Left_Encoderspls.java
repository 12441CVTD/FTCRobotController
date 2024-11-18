package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//you just have to adapt the stuff to 4 wheel and it might work maybe

@Autonomous(name="badecnoder")

public class Left_Encoderspls extends LinearOpMode {


    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;
    //public DcMotor OPL;
    //public DcMotor OPM;
    //public DcMotor OPR;
    //idk what the dead wheels are considered in the driver hub so i just made them this for now
    // OP stands for odometry pod



    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 2000 ;
    static final double     WHEEL_DIAMETER_INCHES   = 1.7 ;
    //these are both right
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //it took me a while to realize this is pi

    @Override


    public void runOpMode() {

        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");
        //OPL = hardwareMap.dcMotor.get("OPL");
        //OPM = hardwareMap.dcMotor.get("OPM");
        //OPR = hardwareMap.dcMotor.get("OPR");

        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//reset encoders during initialization
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //OPL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //OPM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //OPR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //OPL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //OPM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //OPR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorLeft.getCurrentPosition(), motorRight.getCurrentPosition(),
                fL.getCurrentPosition(), bL.getCurrentPosition(), fR.getCurrentPosition(), bR.getCurrentPosition());
        telemetry.update();

        waitForStart();
// i aint gonna mess with anything down here yet lol
//code motion here

        encoderDrive(0.5,  12,  12, 5.0);

    }




    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newfLTarget;
        int newbLTarget;
        int newfRTarget;
        int newbRTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfLTarget = fL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbLTarget = bL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfRTarget = fR.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbRTarget = bR.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);

            fL.setTargetPosition(newfLTarget);
            bL.setTargetPosition(newbLTarget);
            fR.setTargetPosition(newfRTarget);
            bR.setTargetPosition(newbRTarget);

            // Turn On RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fL.setPower(Math.abs(speed));
            bL.setPower(Math.abs(speed));
            fR.setPower(Math.abs(speed));
            bR.setPower(Math.abs(speed));



            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fL.isBusy() && bL.isBusy() && fR.isBusy() && bR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d");
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        fL.getCurrentPosition(),
                        bL.getCurrentPosition(),
                        fR.getCurrentPosition(),
                        bR.getCurrentPosition());
                telemetry.update();
            }



            // Stop all motion;
            fL.setPower(0);
            bL.setPower(0);
            fR.setPower(0);
            bR.setPower(0);

            // Display complete for the driver.
            telemetry.addData("Path", "Complete");
            telemetry.update();


            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }


    }


}


