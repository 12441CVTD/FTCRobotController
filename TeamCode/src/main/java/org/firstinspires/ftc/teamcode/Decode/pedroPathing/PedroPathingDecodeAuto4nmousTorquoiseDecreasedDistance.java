package org.firstinspires.ftc.teamcode.Decode.pedroPathing;


import static org.firstinspires.ftc.teamcode.Decode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Decode.DecodeArms;
import org.firstinspires.ftc.teamcode.Decode.DecodeIntake;

import java.util.TimerTask;


@Autonomous(name = "Blue Close Pedro", group = "Examples")
@Configurable
public class PedroPathingDecodeAuto4nmousTorquoiseDecreasedDistance extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    java.util.Timer timer = new java.util.Timer();
    DecodeArms launcher = null;
    DecodeIntake intake = null;
    private int dbm = 100;
    private Timer pathTimer;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19.5, 123.4, Math.toRadians(315)));

        follower.setMaxPowerScaling(.5);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();

        intake = new DecodeIntake(hardwareMap);
        launcher = new DecodeArms(hardwareMap);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    public int getPathState() {
        return pathState;
    }


    public static class Paths {

        public PathChain Launch0;
        public PathChain Pickup1;
        public PathChain Launch1;
        public PathChain Pickup2;
        public PathChain Launch2;
        public PathChain Pickup3;
        public PathChain Launch3;
        public PathChain Park;

        public Paths(Follower follower) {
            Launch0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 119.400), new Pose(57.800, 85.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                    .build();

            Pickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.800, 85.500),
                                    new Pose(50.768, 83.506),
                                    new Pose(19.453, 83.743)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Launch1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.453, 83.743), new Pose(57.800, 85.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Pickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.800, 85.500),
                                    new Pose(58.359, 57.173),
                                    new Pose(58.122, 59.071),
                                    new Pose(19.453, 59.783)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Launch2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.453, 59.783), new Pose(57.800, 85.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Pickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.800, 85.500),
                                    new Pose(57.885, 29.417),
                                    new Pose(57.885, 36.297),
                                    new Pose(19.453, 35.822)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Launch3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.453, 35.822), new Pose(57.800, 85.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.800, 85.500), new Pose(15.895, 96.079))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }







    public int autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                timer.schedule(new LaunchAuto(), 0);
                timer.schedule(new GateOpen(), 1000);
                follower.followPath(paths.Launch0, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    timer.schedule(new IntakeAuto(), 0);
                    timer.schedule(new GateClose(), 0);
                    follower.followPath(paths.Pickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    timer.schedule(new IntakeAutoOff(), 500);
                    timer.schedule(new GateOpen(), 2000);
                    pathTimer.resetTimer();
                    follower.followPath(paths.Launch1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    timer.schedule(new IntakeAuto(), 0);
                    timer.schedule(new GateClose(), 0);
                    follower.followPath(paths.Pickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    timer.schedule(new IntakeAutoOff(), 500);
                    timer.schedule(new GateOpen(), 3000);
                    follower.followPath(paths.Launch2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    timer.schedule(new IntakeAuto(), 0);
                    timer.schedule(new GateClose(), 0);
                    follower.followPath(paths.Pickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    timer.schedule(new IntakeAutoOff(), 500);
                    timer.schedule(new GateOpen(), 3000);
                    follower.followPath(paths.Launch3,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4){
                    timer.schedule(new LaunchAuto(), 0);
                    follower.followPath(paths.Park, true);
                    setPathState(8);
                }
        }

        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }


    //classes for timer tasks
    public class IntakeAuto extends TimerTask {

        @Override
        public void run() {
            intake.on();
        }
    }

    public class IntakeAutoOff extends TimerTask {

        @Override
        public void run() {
            intake.off();
        }
    }

    public class LaunchAuto extends TimerTask {

        @Override
        public void run() {
            launcher.powAmpMAX();
        }
    }

    public class GateOpen extends TimerTask {

        @Override
        public void run() {
            launcher.gateOpen();
        }
    }

    public class GateClose extends TimerTask {

        @Override
        public void run() {
            launcher.gateClose();
        }
    }


}//closes class