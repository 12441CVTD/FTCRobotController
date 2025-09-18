package org.firstinspires.ftc.teamcode.Decode;
//
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DecodeArms {

    public void init(HardwareMap hwareMap) {
        leftLauncher = hwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hwareMap.get(DcMotor.class, "rightLauncher");
    }

    private DcMotor leftLauncher;
    private DcMotor rightLauncher;

    public void Launch() {
        if (gamepad2.a) {
            leftLauncher.setPower(0.6);
            rightLauncher.setPower(0.6);
        }
    }



}