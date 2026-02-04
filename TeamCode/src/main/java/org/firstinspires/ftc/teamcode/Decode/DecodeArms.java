package org.firstinspires.ftc.teamcode.Decode;
//
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.CRServo;
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
        highGate = hwareMap.get(CRServo.class, "highgate");
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1, 0, 0, 12.5));
        //rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }

    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;
    private Servo gate;
    private CRServo highGate;


    /*public void on() {
            leftLauncher.setPower(-0.55);
            rightLauncher.setPower(0.55);
    }*/

    public void powAmp(){
        leftLauncher.setVelocity(-1542.8);
        rightLauncher.setVelocity(1542.8/*0.58 %pow*/);
        //leftLauncher.setPower(-0.58);
        //rightLauncher.setPower(0.58/*0.58 %pow*/);
    }

    public void powAmpMed(){
        leftLauncher.setVelocity(-1679); // Max velocity: ~2660 maychance
        rightLauncher.setVelocity(1679);
        //leftLauncher.setPower(-0.65);
        //rightLauncher.setPower(0.65/*0.65 %pow*/);
    }

    public void powAmpMAX(){
        leftLauncher.setVelocity(-1755.6);
        rightLauncher.setVelocity(1755.6);
        //leftLauncher.setPower(-0.66);
        //rightLauncher.setPower(0.66/*0.66 %pow*/);
    }

    public void powReversal() {
        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
        //leftLauncher.setPower(0);
        //rightLauncher.setPower(0);
    }

    public double getLeftVelocity(){
        return leftLauncher.getVelocity();
    }

    public double getRightVelocity(){
        return rightLauncher.getVelocity();
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

    /*public void gateClose() {
        gate.setPosition(0.99);
    }*/

    public void gateClose(double position){
        if(gate.getPosition() != position){
            gate.setPosition(position);
        }
    }

    public void highGateOpen () {
        highGate.setPower(-0.2);
    }

    public void highGateClose() {
        highGate.setPower(0);
    }

    public void highGateReverse () {
        highGate.setPower(0.2);
    }

    /*public void off() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }*/



}