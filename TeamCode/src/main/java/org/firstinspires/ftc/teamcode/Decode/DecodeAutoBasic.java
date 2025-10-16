package org.firstinspires.ftc.teamcode.Decode;

import org.firstinspires.ftc.teamcode.Decode.DecodeArms;
import org.firstinspires.ftc.teamcode.Decode.DecodeIntake;
import org.firstinspires.ftc.teamcode.Decode.DecodeMecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;
import java.util.TimerTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Decode_Basic_Auto", group = "Decode")
public class DecodeAutoBasic extends OpMode {

    @Override
    public void init() {
        DecodeArms launcher = new DecodeArms();
        DecodeIntake intake = new DecodeIntake();
        DecodeMecanumDrive chassis = new DecodeMecanumDrive();

        Timer timer = new Timer();
    }

    @Override
    public void loop() {
        
    }
}
