package org.firstinspires.ftc.teamcode.Decode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DecodeIntake {

    public void init(HardwareMap hwaremap) {
        leftIntake = hwaremap.get(CRServo.class, "leftIntake");
        rightIntake = hwaremap.get(CRServo.class, "rightIntake");
    }

    private CRServo leftIntake;
    private CRServo rightIntake;

    public void on() {
        leftIntake.setPower(0.6);
        rightIntake.setPower(0.6);
    }

    public void off() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
    /*telemetry.addData("Initialized",);
    telemetry.update();*/
}
