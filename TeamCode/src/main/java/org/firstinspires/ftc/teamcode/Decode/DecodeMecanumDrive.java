package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DecodeMecanumDrive {
    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor fR = null;
    private DcMotor bR = null;



    public DecodeMecanumDrive(HardwareMap hwMap){
        fL = hwMap.get(DcMotor.class, "frontLeft");
        bL = hwMap.get(DcMotor.class, "frontRight");
        fR = hwMap.get(DcMotor.class, "backLeft");
        bR = hwMap.get(DcMotor.class, "backRight");

        fL.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
    }

        // Wait for the game to start (driver presses START)


        // run until the end of the match (driver presses STOP)


            // Can be used to make toggleable inputs
            //previousGP2.copy(currentGP2);
            //currentGP2.copy(gamepad2);

            // Format: if(currentGP2.*insertInput* && !previousGP2.*insertInput*)

            // Setup a variable for each drive wheel to save power level for telemetry
            //double fLPower;
            //double bLPower;
            //double fRPower;
            //double bRPower;

            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;

            //fLPower    = Range.clip(drive + turn, -0.8, 0.8) ;
            //fRPower   = Range.clip(drive - turn, -0.8, 0.8) ;
            //bLPower    = Range.clip(drive + turn, -0.8, 0.8) ;
            //bRPower   = Range.clip(drive - turn, -0.8, 0.8) ;

        private void setPowers(double fl, double fr, double bl, double br) {
            double maxSpeed = 1.0;

            maxSpeed = Math.max(maxSpeed, Math.abs(fl));
            maxSpeed = Math.max(maxSpeed, Math.abs(fr));
            maxSpeed = Math.max(maxSpeed, Math.abs(bl));
            maxSpeed = Math.max(maxSpeed, Math.abs(br));

            fl /= maxSpeed;
            fr /= maxSpeed;
            bl /= maxSpeed;
            br /= maxSpeed;

            fL.setPower(fl);
            fR.setPower(fr);
            bL.setPower(bl);
            bR.setPower(br);
        }

            public void drive(double right, double forward, double rotate) {
                double fLPower = right + forward + rotate;
                double fRPower = right - forward - rotate;
                double bLPower = right - forward + rotate;
                double bRPower = right + forward - rotate;

                setPowers(fLPower, fRPower, bLPower, bRPower);
            }

            public void fourDrive(double fLPower, double fRPower, double bLPower, double bRPower){
                setPowers(fLPower, fRPower, bLPower, bRPower);
            }



    }


