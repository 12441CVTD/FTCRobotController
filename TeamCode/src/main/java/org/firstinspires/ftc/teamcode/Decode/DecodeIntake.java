package org.firstinspires.ftc.teamcode.Decode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DecodeIntake {

    public DecodeIntake(HardwareMap hwaremap) {
        leftIntake = hwaremap.get(DcMotor.class, "leftIntake");
    }

    private DcMotor leftIntake;

    public void on() {
        leftIntake.setPower(1);
    }

    public void reverse() {
        leftIntake.setPower(-1);
    }

    public void off() {
        leftIntake.setPower(0);
    }
    /*telemetry.addData("Initialized",);
    telemetry.update();*/
}
