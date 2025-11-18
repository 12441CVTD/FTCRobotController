package org.firstinspires.ftc.teamcode.Decode;
//
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class DecodeArms {

    public DecodeArms(HardwareMap hwareMap) {
        leftLauncher = hwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hwareMap.get(DcMotorEx.class, "rightLauncher");
        gate = hwareMap.get(Servo.class, "gate");
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }

    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;
    private Servo gate;


    /*public void on() {
            leftLauncher.setPower(-0.55);
            rightLauncher.setPower(0.55);
    }*/

    public void powAmplification(){
        leftLauncher.setVelocity(-1000);
        rightLauncher.setVelocity(1000);

    }

    public void powAmplificationMAX(){
        leftLauncher.setVelocity(-2500);
        rightLauncher.setVelocity(2500);
    }

    public void powReversal() {
        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
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
        gate.setPosition(0.75);
    }
    public void gateClose() {
        gate.setPosition(0.99);
    }


    /*public void off() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }*/



}