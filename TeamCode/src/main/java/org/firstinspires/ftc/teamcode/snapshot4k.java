//this is gonna mess everything up
//camera stuff
//help
/*# Overview of Camera Interfaces

All camera interfaces implement the common OpenCvCamera interface. Extended functionality specific to each type of camera is exposed in individual interfaces:
 - OpenCvInternalCamera for the Android camera v1 API
   - Offers slightly lower overhead time compared to v2
   - Is **not** currently compatible with FTC Dash Camera stream (but the others are)
 - OpenCvInternalCamera2 for the Android camera v2 API
   - Offers much more fine grained control over the camera sensor compared ot v1, such as ISO, exposure, and focus control.
 - OpenCvWebcam for the FTC SDK's external webcam API

# Overview of Program Flow for Initializing a Camera

### 1. Using a live viewport (or not)

Before you can obtain a camera instance from the camera factory, you must decide whether or not you wish to have a live camera preview displayed on the Robot Controller screen. If so, you can obtain the view ID for the camera monitor container in exactly the same way as is done for Vuforia.
```
int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
```
If not, skip to section 2

Note that even if you choose to use a live viewport, you can programmatically pause it to reduce CPU/battery load using `camera.pauseViewport();` and `webcam.resumeViewport();`

### 2. Creating a Camera Instance Using the Camera Factory

All types of supported cameras are created by invoking the `OpenCvCameraFactory.getInstance().createXYZ(...)` methods.

 - To create an `OpenCvInternalCamera` instance:

 The first parameter specifies the desired camera (FRONT or BACK) and the second (optional) parameter specifies the view ID in which to insert the live preview. If you want to use a live preview, see section 1. If not, simply omit the second parameter.
 ```java
 // With live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

 // Without live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
 ```
 - To create an `OpenCvInternalCamera2` instance:

 The first parameter specifies the desired camera (FRONT or BACK) and the second (optional) parameter specifies the view ID in which to insert the live preview. If you want to use a live preview, see section 1. If not, simply omit the second parameter.
 ```java
 // With live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

 // Without live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK);
 ```

 - To create an `OpenCvWebcam` instance:

 The first parameter specifies the `WebcamName` of the webcam you wish to use. and the second (optional) parameter specifies the view ID in which to insert the live preview. If you want to use a live preview, see section 1. If not, simply omit the second parameter.

 Note: a `WebcamName` can be obtained from the `hardwareMap` like so:
 ```java
 WebcamName webcamName = hardwareMap.get(WebcamName.class, "NAME_OF_CAMERA_IN_CONFIG_FILE")
 ```

 Once you've obtained the `WebcamName`, you can proceed to using the camera factory:
 ```java
 // With live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

 // Without a live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
 ```

### 3. Opening the Camera Device

Now that you've obtained an `OpenCvCamera` instance from the camera factory, the next step is to open the connection to the camera device. There are two methods for doing this:
 - Asynchronously (recommended).

   When opening asynchronously, your thread is not blocked. Instead, you provide a callback in order to be notified when the opening process has been completed. Usually it is in this callback that you'll want to start streaming from the camera (see section 4)

     ```java
     camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
     {
         @Override
         public void onOpened()
         {
             // Usually this is where you'll want to start streaming from the camera (see section 4)
         }
         @Override
         public void onError(int errorCode)
         {
            /*
            * This will be called if the camera could not be opened
            */
    //     }
    // });
     //```
/* - Synchronously (not recommended)

 When opening synchronously, your thread is blocked until the operation is complete.

 ```java
 camera.openCameraDevice();
 ```

### 4. Starting a Streaming Session

After opening the camera, you can start a streaming session by calling `camera.startStreaming(...)`. The first two parameters are the desired width and height of the image stream.

> NOTE: If you specify a resolution which the camera does not support, the program will crash and display an error message will tells you which resolutions ARE supported.
> Commonly supported resolutions include:
>  - 320x240
>  - 640x480
>  - 1280x720
>  - 1920x1080

The third parameter specifies the orientation the camera is being used in. So for instance, if you have a phone mounted in portrait on your robot, or a webcam used in its normal orientation, use `UPRIGHT`. But if you have the phone mounted in landscape on your robot, use `SIDEWAYS_LEFT` or `SIDEWAYS_RIGHT` depending on the specific way you've mounted it. `SIDEWAYS_LEFT` refers to the orientation where the screen is facing toward you in landscape mode and the USB port is facing to the right.
```java
camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
```

> NOTE: If you are using a an internal phone camera in a streaming orientation which does not match the current activity orientation, you will find that the live preview is rendered 90 degrees out from how it "should be". This does NOT impact how frames are delivered to your code. However, if you wish to correct for this, you can call
> ```java
> camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
> ```
> Keep in mind this setting is not applicable when using a webcam. It's only needed when the camera orientation is physically tied to the device orientation, which is the case with internal cameras.
>
> NOTE: Irrespective of whether you're using the `OPTIMIZE_VIEW` policy (but especially if you are), you may want to enable GPU-accelerated rendering for the viewport. This can be done with:
> ```java
> // NOTE: this must be called *before* you call startStreaming(...)
> camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
> ```
> Using GPU acceleration allows for faster render time to the display, which can reduce CPU load and make the preview smoother. It's especially helpful when using `OPTIMIZE_VIEW` because of the rotation that it may perform under the hood.
> However, using GPU acceleration has been observed to occasionally cause crashes in libgles.so / libutils.so on some devices, if the activity orientation is changed. (i.e. you rotate the device w/o rotation lock on) while a streaming session is in flight. Caveat emptor.

### 5. Attaching a Pipeline

A streaming session won't do you much good without any OpenCV processing running on it! [To learn about creating a pipeline, please click here](pipelines_overview.md).
You can attach a pipeline to the camera as follows:
```java
camera.setPipeline(yourPipeline);
```

Pipelines may be set at any point during the camera lifecycle. You can even change pipelines while a streaming session is in flight.

### 6. Closing the Camera / Stopping a Streaming Session

Most of the time there is no need to manually close the camera, as it will be automatically closed for you at the end of your OpMode. However, the `OpenCvCamera` interface does provide methods for closing the camera manually. Please refer to the [JavaDocs](https://javadoc.io/doc/org.openftc/easyopencv/latest/org/openftc/easyopencv/OpenCvCamera.html) for information about these methods.

# Using the Driver Station Camera Preview Feature

For the Control Hub, the DS camera preview feature may prove very helpful for you since it can show the output of your pipeline without requiring an HDMI monitor or use of `scrcpy`.

To use this feature, simply make sure that you've started a streaming session during the OpMode initialization. When running your OpMode, do **NOT** press start. Only press INIT. While the OpMode is in the INIT phase, you can open the overflow menu on the DriverStation and select the "Camera Stream" option. This provides a tap-to-refresh view of the pipeline output. Select `Camera Stream` again to close the preview and continue the OpMode.*/

//ACTUAL CODE TIME
//R
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//package org.firstinspires.ftc.robotcontroller.external.samples;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

//@Autonomous(name="This is for idiot babies", group="Robot")

//public class RobotAutoDriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */
   // private DcMotor fL = null;
 //   private DcMotor fR = null;
 //   private DcMotor bL = null;
 //   private DcMotor bR = null;
//    private ElapsedTime     runtime = new ElapsedTime();

 //   @Override
 //   public void runOpMode() {

        // Initialize the drive system variables.
  //      fL  = hardwareMap.get(DcMotor.class, "fL");
    //    fR  = hardwareMap.get(DcMotor.class, "fR");
      //  bL  = hardwareMap.get(DcMotor.class, "bL");
        //bR  = hardwareMap.get(DcMotor.class, "bR");
//int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "fungus", hardwareMap.appContext.getPackageName());
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
  //      fL.setDirection(DcMotor.Direction.REVERSE);
    //    fR.setDirection(DcMotor.Direction.FORWARD);
      //  bL.setDirection(DcMotor.Direction.REVERSE);
        //bR.setDirection(DcMotor.Direction.FORWARD);
        //fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     //OpenCvCamera fungus = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        // Send telemetry message to signify robot waiting;
       // telemetry.addData("Status", "Ready to run");    //
        //telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();

      // camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//{
  //  @Override
    //public void onOpened()
    //{
        // Usually this is where you'll want to start streaming from the camera (see section 4)
    //}
    //@Override
    //public void onError(int errorCode)
    //{
       /*
       * This will be called if the camera could not be opened
       */
    }
//});
   //  camera.startsteaming(640, 480, OpenCvCameraRotation.UPRIGHT);
     
     // fL.setPower(0);
       // fR.setPower(0);
       // bL.setPower(0);
       // bR.setPower(0);
   // }
   // public void driveWay(double lF, double rF, double lB, double rB, time s){
     //   runtime.reset();
       // while(opModeIsActive() && (runtime.milliseconds() < s)){
         //   fL.setPower(lF);
           // fR.setPower(rF);
          //  bL.setPower(lB);
          //  bR.setPower(rB);
        //}
       // fL.setPower(0);
       // fR.setPower(0);
      //  bL.setPower(0);
        //bR.setPower(0);
   // }
//}
/*package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

private DcMotor fl = null;
private DcMotor fr = null;
private DcMotor bl= null;
private DcMotor br = null;

 private ElapsedTime     runtime = new ElapsedTime();

  @Override
    public void runOpMode() {
fl = hardwareMap.get(DcMotor.class "fl");
fr = hardwareMap.get(DcMotor.class "fr")
bl = hardwareMap.get(DcMotor.class "bl");
br = hardwareMap.get(DcMotor.class "br");

leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

 telemetry.addData("Status", "Ready to run");    
        telemetry.update();

}*/
package org.firstinspires.ftc.teamcode.OpModes.Angle_PID_Tutorial;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Cone Tracker")

public class CameraFusedPID extends LinearOpMode {
    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    Drivetrain drivetrain = new Drivetrain();

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private BNO055IMU imu;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    /** MAKE SURE TO CHANGE THE FOV AND THE RESOLUTIONS ACCORDINGLY **/
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    private static final double FOV = 40;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels


    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Target IMU Angle", getAngleTarget(cX));
            telemetry.addData("Current IMU Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            double power = PIDControl(Math.toRadians(0 + getAngleTarget(cX)), imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
            drivetrain.power(power);
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getAngleTarget(double objMidpoint){
        double midpoint = -((objMidpoint - (CAMERA_WIDTH/2))*FOV)/CAMERA_WIDTH;
        return midpoint;
    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }


}
