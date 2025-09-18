package org.firstinspires.ftc.teamcode.Decode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;

public class DecodeIntake {

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
