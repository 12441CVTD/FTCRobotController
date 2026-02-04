package org.firstinspires.ftc.teamcode.Decode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DecodeIntake {

    public DecodeIntake(HardwareMap hwaremap) {
        intake = hwaremap.get(DcMotor.class, "leftIntake");
    }

    private DcMotor intake;

    public void on() {
        intake.setPower(1);
    }

    public void reverse() {
        intake.setPower(-1);
    }

    public void off() {
        intake.setPower(0);
    }
    /*telemetry.addData("Initialized",);
    telemetry.update();*/
}
