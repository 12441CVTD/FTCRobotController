package org.firstinspires.ftc.teamcode.Decode;

//
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.opencv.core.Point;

@Autonomous

public class DecodeAuto extends LinearOpMode {



    private double x;
    private double y;
    Point cords = new Point(x, y);
    double[] coordinates = {x, y};


    @Override
    public void runOpMode() {

    }
}