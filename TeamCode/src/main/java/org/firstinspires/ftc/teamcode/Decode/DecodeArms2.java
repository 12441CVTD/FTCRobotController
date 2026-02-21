package org.firstinspires.ftc.teamcode.Decode;
//

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;
import java.util.TimerTask;

public class DecodeArms2 {

    public DecodeArms2(HardwareMap hwareMap) {
        launcherUp = hwareMap.get(DcMotorEx.class, "launcherUp");
        launcherDown = hwareMap.get(DcMotorEx.class, "launcherDown");
        transfer = hwareMap.get(DcMotor.class, "transfer");
        gate = hwareMap.get(Servo.class, "gate");
        flipper = hwareMap.get(Servo.class, "flipper");
        turretLeft = hwareMap.get(Servo.class, "tL");
        turretRight = hwareMap.get(Servo.class, "tR");
        launcherUp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(180, 0, 0, 15.567));
        launcherDown.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(180, 0, 0, 15.567));
    }

    private DcMotorEx launcherUp;
    private DcMotorEx launcherDown;
    private DcMotor transfer;
    private Servo turretLeft;
    private Servo turretRight;
    private Servo gate;
    private Servo flipper;

    public void gateClose() {
        gate.setPosition(0.7);
    }

    public void gateOpen() {
        gate.setPosition(0.5);
    }

    public void flipperUp() {
        flipper.setPosition(0.55);
    }

    public void flipperDown() {
        flipper.setPosition(0.0);
    }


    /*public void on() {
            leftLauncher.setPower(-0.55);
            rightLauncher.setPower(0.55);
    }*/

    public void powAmp(){
        launcherUp.setVelocity(-1080/*0.58 %pow*/);
        launcherDown.setVelocity(1080/*0.58 %pow*/);
        //launcherUp.setPower(-0.85);
        //launcherDown.setPower(0.85);
    }

    public void powAmpMed() {
        //launcherUp.setVelocity(-1729/*0.58 %pow*/);
        //launcherDown.setVelocity(1729/*0.65 %pow*/);
        launcherUp.setPower(-0.96);
        launcherDown.setPower(0.96);
    }

    public void powAmpMAX(){
        launcherUp.setVelocity(-1600/*0.58 %pow*/);
        launcherDown.setVelocity(1600);
        //launcherUp.setPower(-1);
        //launcherDown.setPower(1);
    }

    public void limelightLaunch(double v){
        launcherUp.setVelocity(-v);
        launcherDown.setVelocity(v);
    }

    public void powReversal() {
        launcherUp.setVelocity(0);
        launcherDown.setVelocity(0);
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
        turretLeft.setPosition(0.73);
        turretRight.setPosition(0.73);
    }

    public void turretRight(){
        turretLeft.setPosition(0.27);
        turretRight.setPosition(0.27);
    }

    public void turretCenter(){
        turretLeft.setPosition(0.5);
        turretRight.setPosition(0.5);
    }

    public void turretWallAim(){
        turretLeft.setPosition(0.6);
        turretRight.setPosition(0.6);
    }


    public void manualLeft(){
        if(turretLeft.getPosition() <= 0.77 && turretRight.getPosition() <= 0.77) {
            turretLeft.setPosition(turretLeft.getPosition() + 0.0075);
            turretRight.setPosition(turretLeft.getPosition() + 0.0075);
        }
    }

    public void manualRight(){
        if(turretLeft.getPosition() >= 0.25 && turretRight.getPosition() >= 0.25) {
            turretLeft.setPosition(turretLeft.getPosition() - 0.005);
            turretRight.setPosition(turretLeft.getPosition() - 0.005);
        }
    }

    public double getLeftPosition(){
        return turretLeft.getPosition();
    }

    public double getRightPosition(){
        return turretRight.getPosition();
    }


    public void transferOn() {transfer.setPower(1);}
    public void transferOff() {transfer.setPower(0);}
    public void transferReverse() {transfer.setPower(-0.8);}
    public void transferReverseManual() {transfer.setPower(-0.35);}


    public double getUpVelocity(){
        return launcherUp.getVelocity();
    }

    public double getDownVelocity(){
        return launcherDown.getVelocity();
    }


    /*public void off() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }*/

    //Auto Aims
    public void AutoClosePower(){
        launcherUp.setVelocity(-1250);
        launcherDown.setVelocity(1250);
    }

    public void RFPower(){
        launcherUp.setVelocity(-1650);
        launcherDown.setVelocity(1650);
    }

    public void BFPreloadAim(){
        turretLeft.setPosition(0.58);
        turretRight.setPosition(0.58);
    }

    public void BFAutoAim(){
        turretLeft.setPosition(0.58);
        turretRight.setPosition(0.58);
    }

    public void RFPreloadAim(){
        turretLeft.setPosition(0.33);
        turretRight.setPosition(0.33);
    }

    public void RFAutoAim(){
        turretLeft.setPosition(0.33);
        turretRight.setPosition(0.33);
    }

    public void AutoAim(double pos){
        turretLeft.setPosition(pos);
        turretRight.setPosition(pos);
    }

    public void adjustTurret(double delta){
        double left = turretLeft.getPosition();
        double right = turretRight.getPosition();

        double newPos = left + delta;
        newPos = Math.max(0.0, Math.min(1.0, newPos));

        turretLeft.setPosition(newPos);
        turretRight.setPosition(newPos);
    }
}

