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


@Autonomous(name = "Blue Far", group = "Examples")
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
        follower.setStartingPose(new Pose(124, 124, Math.toRadians(217)));

        follower.setMaxPowerScaling(.5);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();

        intake = new DecodeIntake(hardwareMap);
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

        public Paths(Follower follower) {
            Launch1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 13.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();

            Pickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 13.000),
                                    new Pose(54.089, 40.000),
                                    new Pose(60.020, 35.110),
                                    new Pose(20.000, 35.585)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Launch1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 35.585), new Pose(56.000, 13.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();

            Pickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 13.000),
                                    new Pose(53.377, 57.885),
                                    new Pose(67.374, 61.680),
                                    new Pose(20.000, 60.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Launch2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 60.000), new Pose(56.000, 13.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();

            Pickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 13.000),
                                    new Pose(55.987, 83.269),
                                    new Pose(64.290, 85.641),
                                    new Pose(20.000, 84.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Launch3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 84.000), new Pose(56.000, 13.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();
        }
    }



    public int autonomousPathUpdate() {
    switch(pathState) {
        case 0:
            timer.schedule(new LaunchAuto(1), 0);
            follower.followPath(paths.Launch0, true);
            setPathState(1);
            break;
        case 1:
            timer.schedule(new IntakeAuto(.8), 0);

            follower.followPath(paths.Pickup1, true);
            setPathState(2);
            break;
        case 2:


            if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                timer.schedule(new IntakeAutoOff(1.0), 0);
                timer.schedule(new LaunchAuto(0), 0);
                timer.schedule(new GateOpen(0), 4000);
                pathTimer.resetTimer();
                follower.followPath(paths.Launch1,true);
                setPathState(3);
            }
            break;
        case 3:
            if(!follower.isBusy()) {
                timer.schedule(new IntakeAuto(1.0), 0);
                timer.schedule(new GateClose(1.0), 0);
                follower.followPath(paths.Pickup2,true);
                setPathState(4);
            }
            break;
        case 4:
            if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                timer.schedule(new IntakeAutoOff(1.0), 0);
                timer.schedule(new LaunchAuto(0), 0);
                timer.schedule(new GateOpen(0), 4000);
                follower.followPath(paths.Launch2,true);
                setPathState(5);
            }
            break;
        case 5:
            if(!follower.isBusy()) {
            timer.schedule(new IntakeAuto(1.0), 0);
            timer.schedule(new GateClose(1.0), 0);
            follower.followPath(paths.Pickup3,true);
            setPathState(6);
        }
            break;
        case 6:
            if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                timer.schedule(new IntakeAutoOff(1.0), 0);
                timer.schedule(new LaunchAuto(0), 0);
                timer.schedule(new GateOpen(0), 4000);
                follower.followPath(paths.Launch3,true);
                setPathState(7);
            }
            break;
    }

    // Add your state machine Here
    // Access paths with paths.pathName
    // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    return pathState;
}


//classes for timer tasks
public class IntakeAuto extends TimerTask {
    double power;

    public IntakeAuto(double p) {
        this.power = p;
    }

    @Override
    public void run() {
        intake.on();
    }
}

    public class IntakeAutoOff extends TimerTask {
        double power;

        public IntakeAutoOff(double p) {
            this.power = p;
        }

        @Override
        public void run() {
            intake.off();
        }
    }

    public class LaunchAuto extends TimerTask {
        double power;

        public LaunchAuto(double p) {
            this.power = p;
        }

        @Override
        public void run() {
            launcher.powAmpMAX();
        }
    }

    public class GateOpen extends TimerTask {
        double power;

        public GateOpen(double p) {
            this.power = p;
        }

        @Override
        public void run() {
            launcher.gateOpen();
        }
    }

    public class GateClose extends TimerTask {
        double power;

        public GateClose(double p) {
            this.power = p;
        }

        @Override
        public void run() {
            launcher.highGateClosed();
        }
    }


}//closes class