//package org.firstinspires.ftc.teamcode.Decode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//public class limelightTurretAim extends OpMode {
//
//    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
//
//    private TurretMechanismTutorial turret = new TurretMechanismTutorial();
//
//    double[] stepSizes = {0.1, 0.01, 0.0001, 0.0001, 0.00001};
//
//    int stepIndex =2;
//
//    @Override
//    public void init() {
//        aprilTagWebcam.init(hardwareMap, telemtry);
//        turret.init(hardwareMap);
//
//        telemetry.addLine("Initialized all mechanisms");
//    }
//
//    public void start() {
//        turret.resetTimer();
//    }
//
//    @Override
//    public void loop() {
//        //vision logic
//        aprilTagWebcam.update();
//        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
//
//
//        turret.update(id20);
//        // update P and D on fly
//        // B button cycles thru different step sizes for tuning preciscion
//        if (gamepad1.bWasPressed()) {
//            stepIndex = (stepIndex + 1) % stepSizes.length;  // modulo wraps the index back to 0
//        }
//
//        //D-pad left/right adjust P gain
//        if (gamepad1.dpadLeftWasPressed()) {
//            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
//        }
//        if (gamepad1.dpadRightWasPressed()) {
//            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
//        }
//
//        // D-pad up/down adjusts the D gain
//        if (gamepad1.dpadUpWasPressed()) {
//            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
//        }
//        if (gamepad1.dpadDownWasPressed()) {
//            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
//        }
//
//
//        if (id20 !=null) {
//            telemetry.addData( "cur ID", aprilTagWebcam);
//        } else {
//                telemetry.addLine( "no tag detected. Stopping Turret Motor");
//            }
//            telemetry.addLine( "---------------------------");
//            telemetry.addData(" Tuning P", "%.5f (DPAD L/R)", turret.getkP());
//            telemetry.addData(" Tuning D", "%.5f (DPAD U/P)", turret.getkD());
//            telemetry.addData(" Step SIze", "%.5f (B Button)", stepSizes[stepIndex]);
//
//    }
//}
//}
//
//
