package org.firstinspires.ftc.teamcode.Decode;
//
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DecodeArms {

    public DecodeArms(HardwareMap hwareMap) {
        leftLauncher = hwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hwareMap.get(DcMotor.class, "rightLauncher");
        gate = hwareMap.get(Servo.class, "gate");
    }

    private DcMotor leftLauncher;
    private DcMotor rightLauncher;
    private Servo gate;

    /*public void on() {
            leftLauncher.setPower(-0.55);
            rightLauncher.setPower(0.55);
    }*/

    public void powAmplification(){
        leftLauncher.setPower(-0.58);
        rightLauncher.setPower(0.58);
    }

    public void powAmplificationMAX(){
        leftLauncher.setPower(-0.66);
        rightLauncher.setPower(0.66);
    }

    public void powReversal() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }

    /*
    public void powAmplification(int infTech) {
        if(infTech == 0) {
            leftLauncher.setPower(-0.65);
            rightLauncher.setPower(0.65);
        } else if (infTech == 1){
            leftLauncher.setPower(-0.75);
            rightLauncher.setPower(0.75);
        } else if (infTech == 2){
            leftLauncher.setPower(0);
            rightLauncher.setPower(0);
        }
    }

    public void powReversal(int infTech) {
        if(infTech == 0) {
            leftLauncher.setPower(-0.75);
            rightLauncher.setPower(0.75);
        } else if (infTech == 1){
            leftLauncher.setPower(0);
            rightLauncher.setPower(0);
        } else if (infTech == 2){
            leftLauncher.setPower(-0.65);
            rightLauncher.setPower(0.65);
        }
    }

     */

    public void gateOpen() {
        gate.setPosition(1);
    }
    public void gateClose() {
        gate.setPosition(0.75);
    }


    /*public void off() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }*/



}