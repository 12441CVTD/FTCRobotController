package org.firstinspires.ftc.teamcode.Decode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class test{

    public test(HardwareMap hwareMap){
        Object servo = hwareMap.get(CRServo.class, "servo");
    }

        private CRServo servo;

    public void on() {
        servo.setPower(0.6);
    }
    public void off() {
        servo.setPower(0.0);
    }
}
