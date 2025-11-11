package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DecodeActuator {
    public DcMotor actuator = null;

    public DecodeActuator(HardwareMap hwMap) {
        actuator = hwMap.get(DcMotor.class, "actuator");
    }

    public void on() {
        actuator.setPower(1.0);
    }
    public void off() {
        actuator.setPower(0.0);
    }

}
