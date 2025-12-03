package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TestTeleOp extends OpMode {

    test servo = null;

    @Override
    public void init() {
        servo = new test(hardwareMap);
    }


    @Override
    public void loop() {
        if(gamepad1.a) {
            servo.on();
        }
        if(gamepad1.b) {
            servo.off();
        }
    }
}
