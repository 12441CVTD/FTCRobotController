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

import org.firstinspires.ftc.teamcode.Decode.BlueLimelightAutoAim;
import org.firstinspires.ftc.teamcode.Decode.DecodeArms2;
import org.firstinspires.ftc.teamcode.Decode.DecodeIntake;

import java.util.TimerTask;


@Autonomous(name = "Blue Close Pedro V2", group = "Examples")
@Configurable
public class BC extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    java.util.Timer timer = new java.util.Timer();
    DecodeArms2 launcher = null;
    private int dbm = 100;
    private Timer pathTimer;

    BlueLimelightAutoAim vision;
    boolean targetLock = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = ConstantsV2.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24.1, 123, Math.toRadians(135)));

        follower.setMaxPowerScaling(.5);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();

        launcher = new DecodeArms2(hardwareMap);

        vision = new BlueLimelightAutoAim(hardwareMap);
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

        //for limelight
        if(vision.hasTarget()) {
            double tx = vision.getTx();
            double kP = 0.0005; // safe starting value
            targetLock = true;
            if (Math.abs(tx) > 0.5) { // deadband
                launcher.adjustTurret(-kP * tx);

            }
        }
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
                                    new Pose(21.500, 122.500),

                                    new Pose(59.500, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                    .build();

            Pickup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.500, 84.000),

                                    new Pose(15.255, 84.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Launch1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.255, 84.000),

                                    new Pose(59.500, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Pickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.500, 84.000),
                                    new Pose(60.257, 58.596),
                                    new Pose(58.122, 59.071),
                                    new Pose(57.289, 59.759),
                                    new Pose(5.850, 59.783)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Launch2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.850, 59.783),
                                    new Pose(49.400, 58.100),
                                    new Pose(59.500, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Pickup3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.500, 84.000),
                                    new Pose(57.885, 29.417),
                                    new Pose(59.308, 35.822),
                                    new Pose(55.256, 35.137),
                                    new Pose(5.850, 35.822)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Launch3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.850, 35.822),

                                    new Pose(59.500, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.500, 84.000),

                                    new Pose(15.895, 96.079)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();
        }
    }







    public int autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                timer.schedule(new LaunchAuto(), 0);
                timer.schedule(new aim(), 0);
                follower.followPath(paths.Launch0, 0.85, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateOpen(), 500);
                    timer.schedule(new TransferOn(), 700);
                    timer.schedule(new FlipperDown(), 2000);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.Pickup1,0.8, true);
                    timer.schedule(new GateClose(), 0);
                    timer.schedule(new FlipperUp(), 200);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.Launch1,0.85, true);
                    timer.schedule(new TransferOff(), 750);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateOpen(), 500);
                    timer.schedule(new TransferOn(), 700);
                    timer.schedule(new FlipperDown(), 2000);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.Pickup2,0.8, true);
                    timer.schedule(new GateClose(), 0);
                    timer.schedule(new FlipperUp(), 200);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.Launch2,0.85, true);
                    timer.schedule(new TransferOff(), 750);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateOpen(), 500);
                    timer.schedule(new TransferOn(), 700);
                    timer.schedule(new FlipperDown(), 2000);
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.Pickup3, 0.8,true);
                    timer.schedule(new GateClose(), 0);
                    timer.schedule(new FlipperUp(), 200);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.Launch3, 0.85,true);
                    timer.schedule(new TransferOff(), 750);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateOpen(), 500);
                    timer.schedule(new TransferOn(), 700);
                    timer.schedule(new FlipperDown(), 2000);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.Park,0.85,true);
                    timer.schedule(new GateClose(), 0);
                    timer.schedule(new TransferOff(), 0);
                    timer.schedule(new FlipperUp(), 0);
                    timer.schedule(new LaunchAutoOff(),0);
                    setPathState(12);
                }

        }

        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }


    //classes for timer tasks


    public class TransferOn extends TimerTask {

        @Override
        public void run() {
            launcher.transferOn();
        }
    }

    public class TransferOff extends TimerTask {

        @Override
        public void run() {
            launcher.transferOff();
        }
    }

    public class LaunchAuto extends TimerTask {

        @Override
        public void run() {
            launcher.AutoClosePower();
        }
    }

    public class LaunchAutoOff extends TimerTask {

        @Override
        public void run() {
            launcher.powReversal();
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

    public class FlipperUp extends TimerTask {

        @Override
        public void run() {
            launcher.flipperUp();
        }
    }

    public class FlipperDown extends TimerTask {

        @Override
        public void run() {
            launcher.flipperDown();
        }
    }

    public class aim extends TimerTask {

        @Override
        public void run() {
            launcher.turretRight();
        }
    }


}//closes class