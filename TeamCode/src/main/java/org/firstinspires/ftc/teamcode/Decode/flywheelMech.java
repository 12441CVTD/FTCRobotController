package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class flywheelMech {
    private DcMotorEx flywheelMotorR;
    private DcMotorEx flywheelMotorL;
    private double highVelocity = 1150;
    private double lowVelocity = 900;
    private double feederTimeSeconds = 5;
    double curTargetVelocity = highVelocity;

    // Move these into the constructor or methods
    double F = 20;
    double P = 90;

    ElapsedTime feederTimer = new ElapsedTime();
    private LaunchState launchState = LaunchState.IDLE;

    public void FlywheelMotorOn(int i) {
        flywheelMotorR.setVelocity(i);
        flywheelMotorL.setVelocity(i);
    }

    public void on(boolean b) {

    }

    public void off() {
    }

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    public flywheelMech(HardwareMap hardwareMap) {
        flywheelMotorR = hardwareMap.get(DcMotorEx.class, "launcherUp");
        flywheelMotorL = hardwareMap.get(DcMotorEx.class, "launcherDown");


        // Configuration
        flywheelMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void launch(boolean shotRequested) {
        // Calculate velocity and error INSIDE the method
        double curVelocity = flywheelMotorR.getVelocity();
        double error = curTargetVelocity - curVelocity;

        switch (launchState) {
            case IDLE:
                if (shotRequested) launchState = LaunchState.SPIN_UP;
                break;
            case SPIN_UP:
                flywheelMotorR.setVelocity(highVelocity);
                flywheelMotorL.setVelocity(highVelocity);
                if (curVelocity > lowVelocity) launchState = LaunchState.LAUNCH;
                break;
            case LAUNCH:
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > feederTimeSeconds) {
                    FlywheelMotorOff(); // Stop motors after launch
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public void FlywheelMotorOff() {
        flywheelMotorR.setVelocity(0);
        flywheelMotorL.setVelocity(0);
    }

    public double FlywheelMotorVelocity(){
        return flywheelMotorR.getVelocity();
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Actual Velocity", flywheelMotorR.getVelocity());

    }
}
