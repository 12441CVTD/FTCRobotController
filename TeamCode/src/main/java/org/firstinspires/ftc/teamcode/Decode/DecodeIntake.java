package org.firstinspires.ftc.teamcode.Decode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DecodeIntake {

    public void init(HardwareMap hwaremap) {
        leftIntake = hwaremap.get(DcMotor.class, "leftIntake");
        rightIntake = hwaremap.get(DcMotor.class, "rightIntake");
    }

    private DcMotor leftIntake;
    private DcMotor rightIntake;

    public void on() {
        leftIntake.setPower(-0.6);
        rightIntake.setPower(0.6);
    }

    public void off() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
    /*telemetry.addData("Initialized",);
    telemetry.update();*/
}
