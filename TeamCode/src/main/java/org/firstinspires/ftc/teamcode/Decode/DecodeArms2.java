package org.firstinspires.ftc.teamcode.Decode;
//

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DecodeArms2 {

    public DecodeArms2(HardwareMap hwareMap) {
        launcherUp = hwareMap.get(DcMotorEx.class, "launcherUp");
        launcherDown = hwareMap.get(DcMotorEx.class, "launcherDown");
        transfer = hwareMap.get(DcMotor.class, "transfer");
        wGate = hwareMap.get(CRServo.class, "wGate");
        wheel = hwareMap.get(CRServo.class, "wheel");
        turretLeft = hwareMap.get(Servo.class, "tL");
        turretRight = hwareMap.get(Servo.class, "tR");
        launcherDown.setVelocityPIDFCoefficients(1,0,0,0);
        launcherUp.setVelocityPIDFCoefficients(1,0,0,0);
    }

    private DcMotorEx launcherUp;
    private DcMotorEx launcherDown;
    private DcMotor transfer;
    private Servo turretLeft;
    private Servo turretRight;
    private CRServo wGate;
    private CRServo wheel;


    public void wGateOn(double pow) {
        wGate.setPower(pow);
        wheel.setPower(-pow);
    }


    /*public void on() {
            leftLauncher.setPower(-0.55);
            rightLauncher.setPower(0.55);
    }*/

    public void powAmp(){
        //launcherUp.setVelocity(-1542.8/*0.58 %pow*/);
        //launcherDown.setVelocity(1542.8/*0.58 %pow*/);
        launcherUp.setPower(-0.85);
        launcherDown.setPower(0.85);
    }

    public void powAmpMed() {
        //launcherUp.setVelocity(-1729/*0.58 %pow*/);
        //launcherDown.setVelocity(1729/*0.65 %pow*/);
        launcherUp.setPower(-1);
        launcherDown.setPower(1);
    }

    public void powAmpMAX(){
        //launcherUp.setVelocity(-1755.6/*0.58 %pow*/);
        //launcherDown.setVelocity(1755.6);
        launcherUp.setPower(-1);
        launcherDown.setPower(1);
    }

    public void powReversal() {
        launcherUp.setPower(0/*0.58 %pow*/);
        launcherDown.setPower(0);
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


    public void turretLeft(){
        turretLeft.setPosition(0);
        turretRight.setPosition(0);
    }

    public void turretRight(){
        turretLeft.setPosition(1);
        turretRight.setPosition(1);
    }

    public void turretCenter(){
        turretLeft.setPosition(.3);
        turretRight.setPosition(.3);
    }


    public double getLeftPosition(){
        return turretLeft.getPosition();
    }

    public double getRightPosition(){
        return turretRight.getPosition();
    }


    public void transferOn() {transfer.setPower(0.8);}
    public void transferOff() {transfer.setPower(0);}
    public void transferReverse() {transfer.setPower(-0.8);}





    /*public void off() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }*/



}