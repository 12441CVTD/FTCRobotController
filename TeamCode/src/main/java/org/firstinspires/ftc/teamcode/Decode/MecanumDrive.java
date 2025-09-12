package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "MecanumDrive NO USE")
public class MecanumDrive extends LinearOpMode {
    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor fR = null;
    private DcMotor bR = null;

    private Gamepad previousGP2 = new Gamepad();
    private Gamepad currentGP2 = new Gamepad();

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fL  = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL  = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Can be used to make toggleable inputs
            previousGP2.copy(currentGP2);
            currentGP2.copy(gamepad2);

            // Format: if(currentGP2.*insertInput* && !previousGP2.*insertInput*)

            // Setup a variable for each drive wheel to save power level for telemetry
            double fLPower;
            double bLPower;
            double fRPower;
            double bRPower;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            fLPower    = Range.clip(drive + turn, -0.8, 0.8) ;
            fRPower   = Range.clip(drive - turn, -0.8, 0.8) ;
            bLPower    = Range.clip(drive + turn, -0.8, 0.8) ;
            bRPower   = Range.clip(drive - turn, -0.8, 0.8) ;

            fL.setPower(fLPower);
            fR.setPower(fRPower);
            bL.setPower(bLPower);
            bR.setPower(bRPower);

            if (gamepad1.dpad_up) {
                fL.setPower(0.3);
                fR.setPower(0.3);
                bL.setPower(0.3);
                bR.setPower(0.3);
            }

        }
    }
}

