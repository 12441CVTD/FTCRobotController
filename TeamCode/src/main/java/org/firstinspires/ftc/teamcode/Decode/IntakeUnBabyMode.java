package org.firstinspires.ftc.teamcode.Decode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Timer;
import java.util.TimerTask;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.teamcode.constants.TeleopConstants;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeUnBabyMode {

    private CRServo leftIntake;
    private CRServo rightIntake;
    private boolean isOn;

    public void Intaking (){
        if (gamepad2.right_bumper) {
            isOn = true;
        }
        if (gamepad2.left_bumper) {
            isOn = false;
        }

        while (isOn) {
            leftIntake.setPower(1);
            rightIntake.setPower(1);
        }
        while (!isOn) {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }
    /*telemetry.addData("Initialized",);
    telemetry.update();*/
}
