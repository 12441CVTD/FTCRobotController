package org.firstinspires.ftc.teamcode.Decode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DecodeArms {

    private DcMotor leftLauncher;
    private DcMotor rightLauncher;

    public void Launch() {
        if (gamepad2.a) {
            leftLauncher.setPower(0.6);
            rightLauncher.setPower(0.6);
        }
    }



}