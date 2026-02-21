//package org.firstinspires.ftc.teamcode.Decode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.openftc.apriltag.AprilTagDetection;
//
//public class turretMechanism {
//    private DcMotorEx turret;
//
//    private double kP= 0.0001;
//
//    private double kd = 0.0000;
//
//    private double goalx = 0;
//
//    private double lastError = 0;
//
//    private double angleToleranc = 0.2;
//
//    private final double Max_Power = 0.6;
//    private double power = 0;
//
//    private final ElapsedTime timer= new ElapsedTime();
//
//    private void init(HardwareMap) {
//        turret= hwMap.get(DcMotorEx.class, "turret");
//        turret.setMode(DcMotor.RunMode. RUN_WITHOUT_ENCODER);
//
//    }
//    public void setkP(double newKP){
//        kP= newKP;
//    }
//
//    public double getkP() {
//        return kP;
//    }
//    public void setkD(double newKD) {
//        kD = newKD;
//    }
//
//    public double getkD() {
//        return kD;
//    }
//    public void resetTimer(){
//        timer.reset();
//    }
//
//    public void update(AprilTagDetection curlID) {
//        double deltaTime = timer.seconds();
//        timer.reset();
//
//        if(curlID == null) {
//            turret.setPower(0);
//            lastError = 0;
//            return;
//
//        }
//        double error = goalx -curlID.ftcPose.bearing;
//        double pTerm= error * kP;
//
//        double dTerm= 0;
//        if (deltaTime >0) {
//            dTerm= ((error- lastError) /deltaTime) * kD;
//        }
//        if (Math.abs(error) < angleTolerance) {
//            power = 0;
//        } else {
//            power = Range.clip( pterm + dTerm, -MAX_POWER, MAX_POWER);
//        }
//    }
//}
//
