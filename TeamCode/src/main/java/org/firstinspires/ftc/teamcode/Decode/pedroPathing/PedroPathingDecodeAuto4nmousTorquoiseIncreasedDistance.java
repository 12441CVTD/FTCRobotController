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
import java.util.TimerTask;


import org.firstinspires.ftc.teamcode.Decode.DecodeArms;
import org.firstinspires.ftc.teamcode.Decode.DecodeIntake;



@Autonomous(name = "Blue Far Pedro", group = "Examples")
@Configurable
public class PedroPathingDecodeAuto4nmousTorquoiseIncreasedDistance extends OpMode {
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
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(270)));


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
        panelsTelemetry.debug("Timer", pathTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("IsBusy", follower.isBusy());
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
            Launch0 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(56.000, 13.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(287))

                    .build();

            Pickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 13.000),
                                    new Pose(54.089, 40.000),
                                    new Pose(60.020, 35.110),
                                    new Pose(7.664, 35.111)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Launch1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(7.664, 35.111),

                                    new Pose(56.000, 13.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(287))

                    .build();

            Pickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 13.000),
                                    new Pose(53.377, 57.885),
                                    new Pose(67.374, 61.680),
                                    new Pose(7.664, 58.577)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Launch2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(7.664, 58.577),

                                    new Pose(56.000, 13.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(287))

                    .build();

            Pickup3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 13.000),
                                    new Pose(55.987, 83.269),
                                    new Pose(64.290, 85.641),
                                    new Pose(14.306, 83.763)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Launch3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.306, 83.763),

                                    new Pose(56.000, 13.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(287))

                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 13.000),

                                    new Pose(38.906, 13.522)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();
        }
    }





    public int autonomousPathUpdate() {
    switch(pathState) {
        case 0:
            timer.schedule(new LaunchAuto(), 0);
            timer.schedule(new IntakeAuto(), 2000);
            timer.schedule(new GateOpen(), 1000);
            timer.schedule(new HighGateOpen(), 1500);
            follower.followPath(paths.Launch0, true);
            setPathState(1);
            break;
        case 1:
            if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                timer.schedule(new GateClose(), 0);
                timer.schedule(new HighGateClose(), 3500);
                pathTimer.resetTimer();
                follower.followPath(paths.Pickup1, true);
                setPathState(2);
            }
            break;
        case 2:
            if(!follower.isBusy()){
                timer.schedule(new IntakeAutoOff(), 500);
                timer.schedule(new GateOpen(), 5500);
                timer.schedule(new HighGateOpen(), 4400);
                timer.schedule(new IntakeAuto(), 6000);
                pathTimer.resetTimer();
                follower.followPath(paths.Launch1,true);
                setPathState(7);
            }
            break;
        case 3:
            if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 7) {
                timer.schedule(new IntakeAuto(), 0);
                timer.schedule(new GateClose(), 0);
                pathTimer.resetTimer();
                follower.followPath(paths.Pickup2,true);
                setPathState(4);
            }
            break;
        case 4:
            if(!follower.isBusy()) {
                timer.schedule(new IntakeAutoOff(), 500);
                timer.schedule(new GateOpen(), 6000);
                timer.schedule(new IntakeAuto(), 6500);
                pathTimer.resetTimer();
                follower.followPath(paths.Launch2,true);
                setPathState(7);
            }
            break;
        case 5:
            if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 8) {
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
            if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 8){
                timer.schedule(new LaunchAuto(), 0);
                timer.schedule(new IntakeAutoOff(), 5000);
                timer.schedule(new HighGateClose(), 5000);
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
            launcher.gateClose(0.99);
        }
    }

        public class HighGateOpen extends TimerTask {

        @Override
        public void run() {
            launcher.highGateOpen();
        }
    }


    public class HighGateClose extends TimerTask {

        @Override
        public void run() {
            launcher.highGateClose();
        }
    }


}//closes class