package org.firstinspires.ftc.teamcode.Decode;
//

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DecodeArms2 {

    public DecodeArms2(HardwareMap hwareMap) {
        launcher = hwareMap.get(DcMotor.class, "leftLauncher");
        transferF = hwareMap.get(CRServo.class, "setF");
        TransferS = hwareMap.get(CRServo.class, "setS");
    }

    private DcMotor launcher;
    private CRServo transferF;
    private CRServo TransferS;

    /*public void on() {
            leftLauncher.setPower(-0.55);
            rightLauncher.setPower(0.55);
    }*/

    public void powAmplification(){
        launcher.setPower(-0.6);
    }

    public void powAmplificationMAX(){
        launcher.setPower(-0.66);
    }

    public void powReversal() {
        launcher.setPower(0);
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

    public void transferFOn() {transferF.setPower(1);}
    public void transferFOff() {transferF.setPower(0);}
    public void transferFR() {transferF.setPower(-1);}
    public void transferSOn() {TransferS.setPower(1);}
    public void transferSOff() {TransferS.setPower(0);}
    public void transferSR() {transferF.setPower(-1);}




    /*public void off() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }*/



}