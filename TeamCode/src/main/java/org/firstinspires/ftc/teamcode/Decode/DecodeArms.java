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

    /*public void on() {
            leftLauncher.setPower(-0.55);
            rightLauncher.setPower(0.55);
    }*/

    public void powAmplification(int infTech) {
        if(infTech == 0) {
            leftLauncher.setPower(-0.55);
            rightLauncher.setPower(0.55);
        } else if (infTech == 1){
            leftLauncher.setPower(0);
            rightLauncher.setPower(0);
        }
    }

    public void powReversal(int infTech) {
        if(infTech == 0) {
            leftLauncher.setPower(-0.55);
            rightLauncher.setPower(0.55);
        } else if (infTech == 1){
            leftLauncher.setPower(0);
            rightLauncher.setPower(0);
        }
    }


    /*public void off() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }*/



}