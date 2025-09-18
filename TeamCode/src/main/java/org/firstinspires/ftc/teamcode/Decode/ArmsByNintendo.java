package org.firstinspires.ftc.teamcode.Decode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Timer;
import java.util.TimerTask;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.constants.TeleopConstants;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmsByNintendo {

    private DcMotor leftLauncher;
    private DcMotor rightLauncher;

    public void Launch() {
        if (gamepad2.a) {
            leftLauncher.setPower(0.6);
            rightLauncher.setPower(0.6);
        }
    }



}