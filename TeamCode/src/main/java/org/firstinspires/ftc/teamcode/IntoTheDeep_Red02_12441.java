//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
////
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import com.pedropathing.util.Timer;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.FConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.LConstants;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.constants.AutoConstants;
//import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
//
//
//import java.util.TimerTask;
//
//@Disabled
//@Autonomous(name="Right_Path2", group="IntoTheDeep")
//
//public class IntoTheDeep_Red02_12441 extends LinearOpMode {
//
//    //PedroPathing
//    private Follower follower;
//    int pathState;
//
//
//    private Path scorePreload, park;
//    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
//    private Timer time = new Timer(), times = new Timer();
//
//    private final Pose startPose = new Pose(31.4, 121.1, 0);
//
//
//    //Normalcy
//    private DcMotor fL = null;
//    private DcMotor fR = null;
//    private DcMotor bL = null;
//    private DcMotor bR = null;
//
//    private DcMotor lArm = null;
//    private DcMotor rArm = null;
//
//    private Servo lElbow = null;
//    private Servo rElbow = null;
//    private Servo claw = null;
//    private Servo wrist = null;
//
//    private ElapsedTime runtime = new ElapsedTime();
//    private Timer timer = new Timer();
//
//
//
//    private boolean around = false;
//
//    private int[] LiftConstants = AutoConstants.SpecimenLiftConstants;
//    private double[] ClawConstants = AutoConstants.ClawConstants;
//
//    class claw extends TimerTask{
//        private double position;
//
//        public claw(double position){
//            this.position = position;
//        }
//
//        public void run(){
//            claw.setPosition(position);
//        }
//
//    }
//
//    class wrist extends TimerTask{
//        private double position;
//
//        public wrist(double position){
//            this.position = position;
//        }
//
//        public void run(){
//            wrist.setPosition(position);
//        }
//    }
//
//    class lift extends TimerTask{
//        private int position;
//        private double power;
//
//        public lift(int position, double power){
//            this.position = position;
//            this.power = power;
//        }
//
//        public void run(){
//
//            lArm.setTargetPosition(position);
//            rArm.setTargetPosition(position);
//
//            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            lArm.setPower(power);
//            rArm.setPower(power);
//
//        }
//    }
//
//    class elbow extends TimerTask{
//        double pos;
//
//        public elbow(double position){
//            pos = position;
//        }
//
//        public void run(){
//            lElbow.setPosition(pos);
//            rElbow.setPosition(pos);
//        }
//
//
//    }
//
//    public void runOpMode() {
//
//        // Initialize the drive system variables.
//        fL = hardwareMap.get(DcMotor.class, "fL");
//        fR = hardwareMap.get(DcMotor.class, "fR");
//        bL = hardwareMap.get(DcMotor.class, "bL");
//        bR = hardwareMap.get(DcMotor.class, "bR");
//
//        lElbow = hardwareMap.get(Servo.class, "lElbow");
//        rElbow = hardwareMap.get(Servo.class, "rElbow");
//        claw = hardwareMap.get(Servo.class, "claw");
//        wrist = hardwareMap.get(Servo.class, "wrist");
//
//        // (arm) encoder stuff
//        lArm = hardwareMap.dcMotor.get("lArm");
//        rArm = hardwareMap.dcMotor.get("rArm");
//
//        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        lArm.setDirection(DcMotor.Direction.REVERSE);
//        rArm.setDirection(DcMotor.Direction.FORWARD);
//
//
//        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
//        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        fL.setDirection(DcMotor.Direction.FORWARD);
//        fR.setDirection(DcMotor.Direction.FORWARD);
//        bL.setDirection(DcMotor.Direction.REVERSE);
//        bR.setDirection(DcMotor.Direction.REVERSE);
//        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        lElbow.setDirection(Servo.Direction.FORWARD);
//        rElbow.setDirection(Servo.Direction.REVERSE);
//
//        claw.setDirection(Servo.Direction.FORWARD);
//        wrist.setDirection(Servo.Direction.FORWARD);
//
//        lElbow.setPosition(0);
//        rElbow.setPosition(0);
//
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(20, -60, Math.PI /2));
//
//        waitForStart();
//
//        lArm.setTargetPosition(0);
//        rArm.setTargetPosition(0);
//
//        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        lElbow.setPosition(0);
//        rElbow.setPosition(0);
//        wrist.setPosition(0.5);
//        claw.setPosition(0.43);
//
//        while(opModeIsActive()){
//
//            //GoGoGo(drive);
//
//            sleep(15000);
//            break;
//
//        }
//
//
///*
//        fL.setPower(0);
//        fR.setPower(0);
//        bL.setPower(0);
//        bR.setPower(0);
//*/
//    }
//
//    //Methods
//    public class autoUpdate implements Action{
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            switch (pathState) {
//                case 0:
//                    follower.followPath(scorePreload);
//                    setPath(1);
//                    break;
//                case 1:
//
//                /* You could check for
//                - Follower State: "if(!follower.isBusy() {}"
//                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//                - Robot Position: "if(follower.getPose().getX() > 36) {}"
//                */
//
//                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                    if(!follower.isBusy()) {
//                        /* Score Preload */
//
//                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                        follower.followPath(grabPickup1,true);
//                        setPath(2);
//                    }
//                    break;
//                case 2:
//                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                    if(!follower.isBusy()) {
//                        /* Grab Sample */
//
//                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                        follower.followPath(scorePickup1,true);
//                        setPath(3);
//                    }
//                    break;
//                case 3:
//                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                    if(!follower.isBusy()) {
//                        /* Score Sample */
//
//                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                        follower.followPath(grabPickup2,true);
//                        setPath(4);
//                    }
//                    break;
//                case 4:
//                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                    if(!follower.isBusy()) {
//                        /* Grab Sample */
//
//                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                        follower.followPath(scorePickup2,true);
//                        setPath(5);
//                    }
//                    break;
//                case 5:
//                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                    if(!follower.isBusy()) {
//                        /* Score Sample */
//
//                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                        follower.followPath(grabPickup3,true);
//                        setPath(6);
//                    }
//                    break;
//                case 6:
//                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                    if(!follower.isBusy()) {
//                        /* Grab Sample */
//
//                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                        follower.followPath(scorePickup3, true);
//                        setPath(7);
//                    }
//                    break;
//                case 7:
//                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                    if(!follower.isBusy()) {
//                        /* Score Sample */
//
//                        /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                        follower.followPath(park,true);
//                        setPath(8);
//                    }
//                    break;
//                case 8:
//                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                    if(!follower.isBusy()) {
//                        /* Level 1 Ascent */
//
//                        /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                        setPath(-1);
//                    }
//                    break;
//            }
//            return false;
//        }
//    }
//    public void setPath(int p){
//        pathState = p;
//        times.resetTimer();
//    }
//
//    //Arm Actions
//    /*
//    public class Kickback implements Action{
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(!around){
//                around = !around;
//                timer.schedule(new elbow(0.405), 0);
//                timer.schedule(new wrist(0.34), 0);
//                timer.schedule(new elbow(0.55), 600);
//                timer.schedule(new elbow(0.68), 1300);
//            }
//            else{
//                around = !around;
//                timer.schedule(new elbow(0.405), 0);
//                timer.schedule(new wrist(0.5), 0);
//                timer.schedule(new elbow(0.2025), 700);
//                timer.schedule(new elbow(0), 1250);
//            }
//            return false;
//        }
//
//    }
//    */
//
//    //Claw Actions
//
//    public class OpenClaw implements Action{
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            claw.setPosition(ClawConstants[0]);
//            return false;
//        }
//
//    }
//
//    public class CloseClaw implements Action{
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            claw.setPosition(ClawConstants[1]);
//            return false;
//        }
//    }
//
//    //Lift Actions
//
//    public class UpLift implements Action{
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            lArm.setTargetPosition(LiftConstants[2]);
//            rArm.setTargetPosition(LiftConstants[2]);
//            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            lArm.setPower(1);
//            rArm.setPower(1);
//            return false;
//        }
//    }
//
//    public class toMid implements Action{
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket){
//            lArm.setTargetPosition(LiftConstants[1]);
//            rArm.setTargetPosition(LiftConstants[1]);
//            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            lArm.setPower(1);
//            rArm.setPower(1);
//            return false;
//        }
//
//    }
//
//    public class DownShift implements Action{
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket){
//            lArm.setTargetPosition(LiftConstants[0]);
//            rArm.setTargetPosition(LiftConstants[0]);
//            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            lArm.setPower(1);
//            rArm.setPower(1);
//            return false;
//        }
//
//    }
//    // Big Actions
//
//    /*
//    public class GrabFromGround implements Action{
//        TrajectoryActionBuilder actionBuilder;
//
//        public GrabFromGround(MecanumDrive drive){
//            actionBuilder = drive.actionBuilder(drive.pose);
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            Actions.runBlocking(new SequentialAction(actionBuilder.afterTime(0, new CloseClaw()).build(),
//                                                    actionBuilder.afterTime(0.2, new Kickback()).build(),
//                                                    actionBuilder.afterTime(1.8, new OpenClaw()).build()));
//            return false;
//        }
//    }
//    */
//
//    public class SpecimenPlace implements Action{
//        TrajectoryActionBuilder actionBuilder;
//        double variance;
//        public SpecimenPlace(MecanumDrive drive, double diff){
//            actionBuilder = drive.actionBuilder(drive.pose);
//            variance = diff;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            //Actions.runBlocking(actionBuilder.);
//            return false;
//        }
//    }
//
//    // Merriment
//
//    public class path implements Action{
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            return false;
//        }
//    }
//    private void notPedro(MecanumDrive drive){
//        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(drive.pose);
//        follower.update();
//        Actions.runBlocking(new SequentialAction(actionBuilder.afterTime(0, new autoUpdate()).build()));
//    }
///*
//    private void GoGoGo(MecanumDrive drive){
//        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(drive.pose);
//
//        Actions.runBlocking(new SequentialAction(new ParallelAction(actionBuilder.afterTime(0, new UpLift()).build(), actionBuilder.afterTime(0.05, actionBuilder.strafeTo(new Vector2d(0, -32)).build()).build()),
//                                                    new ParallelAction(new toMid(), actionBuilder.afterTime(0.25, new OpenClaw()).build())));
//
//        Actions.runBlocking(new SequentialAction(new ParallelAction(actionBuilder.strafeTo(new Vector2d(48, -36)).build(), actionBuilder.afterTime(0.85, new DownShift()).build()),
//                                                    actionBuilder.afterTime(0.1, new GrabFromGround(drive)).build()));
//
//        Actions.runBlocking(new SequentialAction(new ParallelAction(actionBuilder.strafeTo(new Vector2d(59, -36)).build(), actionBuilder.afterTime(0.1, new Kickback()).build()),
//                                                    actionBuilder.afterTime(0, new GrabFromGround(drive)).build()));
//
//        actionBuilder = drive.actionBuilder(drive.pose);
//
//        Actions.runBlocking(new SequentialAction(actionBuilder.strafeTo(new Vector2d(60, -48)).build(),
//                                                 actionBuilder.afterTime(0, new CloseClaw()).build()));
//
//        Actions.runBlocking(new ParallelAction(actionBuilder.strafeTo(new Vector2d(5, -32)).build(),
//                                                actionBuilder.afterTime(0, new Kickback()).build(),
//                                                actionBuilder.afterTime(0, new UpLift()).build()));
//
//    }
//*/
//
//}
