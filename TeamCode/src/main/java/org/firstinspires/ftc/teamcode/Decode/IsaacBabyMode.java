package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class IsaacBabyMode extends OpMode {

 private DcMotor fL;
 private DcMotor fR;
 private DcMotor bL;
 private DcMotor bR;

 private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        fR = hardwareMap.get(DcMotor.class, "fR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;
        double fLPower;
        double bLPower;
        double fRPower;
        double bRPower;
        double elapsedSeconds = runtime.seconds();

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;

        boolean is360ing = false;

        fLPower = Range.clip(drive + turn, -0.8, 0.8) ;
        fRPower = Range.clip(drive - turn, -0.8, 0.8) ;
        bLPower = Range.clip(drive + turn, -0.8, 0.8) ;
        bRPower = Range.clip(drive - turn, -0.8, 0.8) ;


        if (gamepad1.a && !is360ing) {
            is360ing = true;
            resetRuntime();
            while (is360ing && elapsedSeconds <= 3.0) {
                fLPower = Range.clip(drive + turn, -0.8, 0.8);
                fRPower = Range.clip(drive - turn, -0.8, 0.8);
                bLPower = Range.clip(drive + turn, -0.8, 0.8);
                bRPower = Range.clip(drive - turn, -0.8, 0.8);

                drive = 0;
                turn = 0.8;
                if (elapsedSeconds >= 3.0) {
                    is360ing = false;
                }
            }
        }

        fL.setPower(leftPower);
        bL.setPower(leftPower);
        fR.setPower(rightPower);
        bR.setPower(rightPower);
    }
}
