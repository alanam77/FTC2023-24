/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Scan Sample", group = "Concept")
//@Disabled
public class ScanningSample extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor LeftRear;
    private DcMotor RightRear;
    private DcMotor arm;
    private DcMotor lift;
    private Servo s1;
    private CRServo s2;
    private Servo s3;
    private Servo s4;
    IMU imu;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "rocketModel.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
       "Rocket",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    YawPitchRollAngles orientation;
    ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 5.0/7.0;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    @Override
    public void runOpMode() {



        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".


        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        LeftRear = hardwareMap.get(DcMotor.class, "left_rear");
        RightRear = hardwareMap.get(DcMotor.class, "right_rear");
        lift = hardwareMap.get(DcMotor.class, "lift");
        arm = hardwareMap.get(DcMotor.class, "arm");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        imu = hardwareMap.get(IMU.class, "imu");

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setDirection(DcMotor.Direction.FORWARD);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Now initialize the IMU with this mounting orientation
        String position = "RIGHT";
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        while (opModeInInit()) {
            telemetry.clear();
            if(telemetryTfod() > 0 && telemetryTfod() < 300){
                position = "LEFT";
                telemetry.addData("Position LEFT: ",telemetryTfod());
            }
            else if(telemetryTfod() > 300){
                position = "MID";
                telemetry.addData("Position MID: ",telemetryTfod());
            }
            else{
                position = "RIGHT";
                telemetry.addData("Position RIGHT: ",telemetryTfod());
            }
            telemetry.update();
        }
        visionPortal.close();
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw: ",orientation.getYaw(AngleUnit.DEGREES));
            //Call RightStrafe to strafe right at a certain angle(Angle is Relative to starting Angle)
            RightStrafe(0,0.2,5,5);
            //Call Forward to move forward at a certain angle(Angle is relative to starting Angle)
            Forward(0,0.2,15,3);

            if(position == "LEFT"){
                break;
            }
            else if(position == "MID"){
                break;
            }
            else{
                break;
            }
        }
        // Save more CPU resources when camera is no longer needed.
    }

        /**
         * Initialize the TensorFlow Object Detection processor.
         */
        public void initTfod () {

            // Create the TensorFlow processor by using a builder.
            tfod = new TfodProcessor.Builder()

                    // With the following lines commented out, the default TfodProcessor Builder
                    // will load the default model for the season. To define a custom model to load,
                    // choose one of the following:
                    //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                    //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                    .setModelAssetName(TFOD_MODEL_ASSET)
                    //.setModelFileName(TFOD_MODEL_FILE)

                    // The following default settings are available to un-comment and edit as needed to
                    // set parameters for custom models.
                    .setModelLabels(LABELS)
                    //.setIsModelTensorFlow2(true)
                    //.setIsModelQuantized(true)
                    //.setModelInputSize(300)
                    //.setModelAspectRatio(16.0 / 9.0)

                    .build();

            // Create the vision portal by using a builder.
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set the camera (webcam vs. built-in RC phone camera).
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            // Choose a camera resolution. Not all cameras support all resolutions.
            //builder.setCameraResolution(new Size(640, 480));

            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            //builder.enableLiveView(true);

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            //builder.setAutoStopLiveView(false);

            // Set and enable the processor.
            builder.addProcessor(tfod);

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();

            // Set confidence threshold for TFOD recognitions, at any time.
            tfod.setMinResultConfidence(0.60f);

            // Disable or re-enable the TFOD processor at any time.
            //visionPortal.setProcessorEnabled(tfod, true);

        }   // end method initTfod()

        /**
         * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
         */
        private double telemetryTfod () {

            List<Recognition> currentRecognitions = tfod.getRecognitions();
//            telemetry.addData("# Objects Detected", currentRecognitions.size());
            double x = 0;
            double y = 0;
            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;

//                telemetry.addData("", " ");
//                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//                telemetry.addData("- Position", "%.0f / %.0f", x, y);
//                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            }   // end for() loop
            return x;
        }   // end method telemetryTfod()
        public void Forward ( double DesiredAngle, double speed, double Inches,
        double timeoutS){
            int newLeftTarget;
            int newRightTarget;
            int newLeftTarget2;
            int newRightTarget2;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = LeftFront.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = RightFront.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftTarget2 = LeftRear.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget2 = RightRear.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

                LeftFront.setTargetPosition(newLeftTarget);
                RightFront.setTargetPosition(newRightTarget);
                LeftRear.setTargetPosition(newLeftTarget2);
                RightRear.setTargetPosition(newRightTarget2);

                // Turn On RUN_TO_POSITION
                LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (LeftFront.isBusy() && RightFront.isBusy() && LeftRear.isBusy() && RightRear.isBusy())) {
                    double YawAngle = orientation.getYaw(AngleUnit.DEGREES);
                    if (YawAngle < (DesiredAngle - .5)) {
                        //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                        //A negative YawAngle is a clockwise rotation for the robot.
                        LeftFront.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        RightFront.setPower(speed);
                        LeftRear.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        RightRear.setPower(speed);
                    } else if (YawAngle > (DesiredAngle + .5)) {
                        //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                        //A positive YawAngle is a counter-clockwise rotation for the robot.
                        LeftFront.setPower(speed);
                        RightFront.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        LeftRear.setPower(speed);
                        RightRear.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                    } else {
                        LeftFront.setPower(speed);
                        RightFront.setPower(speed);
                        LeftRear.setPower(speed);
                        RightRear.setPower(speed);
                    }
                    telemetry.addData("Angle is", YawAngle);
                    telemetry.addData("Motor Power is", "At %7f :%7f :%7f :%7f", LeftFront.getPower(), RightFront.getPower(), LeftRear.getPower(), RightRear.getPower());
                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftTarget2, newRightTarget2);
                    telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            LeftFront.getCurrentPosition(),
                            RightFront.getCurrentPosition(),
                            LeftRear.getCurrentPosition(),
                            RightRear.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                sleep(50);
                LeftFront.setPower(0);
                RightFront.setPower(0);
                LeftRear.setPower(0);
                RightRear.setPower(0);

                // Turn off RUN_TO_POSITION
                LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        }
        public void Reverse ( double DesiredAngle, double speed, double Inches,
        double timeoutS){
            int newLeftTarget;
            int newRightTarget;
            int newLeftTarget2;
            int newRightTarget2;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = LeftFront.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = RightFront.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
                newLeftTarget2 = LeftRear.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
                newRightTarget2 = RightRear.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);

                LeftFront.setTargetPosition(newLeftTarget);
                RightFront.setTargetPosition(newRightTarget);
                LeftRear.setTargetPosition(newLeftTarget2);
                RightRear.setTargetPosition(newRightTarget2);

                // Turn On RUN_TO_POSITION
                LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (LeftFront.isBusy() && RightFront.isBusy() && LeftRear.isBusy() && RightRear.isBusy())) {
                    double YawAngle = orientation.getYaw(AngleUnit.DEGREES);
                    if (YawAngle < (DesiredAngle - .5)) {
                        //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                        //A negative YawAngle is a clockwise rotation for the robot.
                        LeftFront.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        RightFront.setPower(speed);
                        LeftRear.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        RightRear.setPower(speed);
                    } else if (YawAngle > (DesiredAngle + .5)) {
                        //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                        //A positive YawAngle is a counter-clockwise rotation for the robot.
                        LeftFront.setPower(speed);
                        RightFront.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        LeftRear.setPower(speed);
                        RightRear.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                    } else {
                        LeftFront.setPower(speed);
                        RightFront.setPower(speed);
                        LeftRear.setPower(speed);
                        RightRear.setPower(speed);
                    }
                    telemetry.addData("Angle is", YawAngle);
                    telemetry.addData("Motor Power is", "At %7f :%7f :%7f :%7f", LeftFront.getPower(), RightFront.getPower(), LeftRear.getPower(), RightRear.getPower());
                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftTarget2, newRightTarget2);
                    telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            LeftFront.getCurrentPosition(),
                            RightFront.getCurrentPosition(),
                            LeftRear.getCurrentPosition(),
                            RightRear.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                sleep(50);
                LeftFront.setPower(0);
                RightFront.setPower(0);
                LeftRear.setPower(0);
                RightRear.setPower(0);

                // Turn off RUN_TO_POSITION
                LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        }
        public void LeftStrafe ( double DesiredAngle, double speed, double strafeInches,
        double timeoutS){
            int newLeftTarget;
            int newRightTarget;
            int newLeftTarget2;
            int newRightTarget2;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = LeftFront.getCurrentPosition() - (int) (strafeInches * COUNTS_PER_INCH);
                newRightTarget = RightFront.getCurrentPosition() + (int) (strafeInches * COUNTS_PER_INCH);
                newLeftTarget2 = LeftRear.getCurrentPosition() + (int) (strafeInches * COUNTS_PER_INCH);
                newRightTarget2 = RightRear.getCurrentPosition() - (int) (strafeInches * COUNTS_PER_INCH);

                LeftFront.setTargetPosition(newLeftTarget);
                RightFront.setTargetPosition(newRightTarget);
                LeftRear.setTargetPosition(newLeftTarget2);
                RightRear.setTargetPosition(newRightTarget2);

                // Turn On RUN_TO_POSITION
                LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (LeftFront.isBusy() && RightFront.isBusy() && LeftRear.isBusy() && RightRear.isBusy())) {
                    double YawAngle = orientation.getYaw(AngleUnit.DEGREES);
                    if (YawAngle < (DesiredAngle - .5)) {
                        //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                        //A negative YawAngle is a clockwise rotation for the robot.
                        LeftFront.setPower(speed);
                        RightFront.setPower(speed);
                        LeftRear.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        RightRear.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                    } else if (YawAngle > (DesiredAngle + .5)) {
                        //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                        //A positive YawAngle is a counter-clockwise rotation for the robot.
                        LeftFront.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        RightFront.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        LeftRear.setPower(speed);
                        RightRear.setPower(speed);
                    } else {
                        LeftFront.setPower(speed);
                        RightFront.setPower(speed);
                        LeftRear.setPower(speed);
                        RightRear.setPower(speed);
                    }
                    telemetry.addData("Angle is", YawAngle);
                    telemetry.addData("Motor Power is", "At %7f :%7f :%7f :%7f", LeftFront.getPower(), RightFront.getPower(), LeftRear.getPower(), RightRear.getPower());
                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftTarget2, newRightTarget2);
                    telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            LeftFront.getCurrentPosition(),
                            RightFront.getCurrentPosition(),
                            LeftRear.getCurrentPosition(),
                            RightRear.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                sleep(50);
                LeftFront.setPower(0);
                RightFront.setPower(0);
                LeftRear.setPower(0);
                RightRear.setPower(0);

                // Turn off RUN_TO_POSITION
                LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        }
        public void RightStrafe ( double DesiredAngle, double speed, double strafeInches,
        double timeoutS){
            int newLeftTarget;
            int newRightTarget;
            int newLeftTarget2;
            int newRightTarget2;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = LeftFront.getCurrentPosition() + (int) (strafeInches * COUNTS_PER_INCH);
                newRightTarget = RightFront.getCurrentPosition() - (int) (strafeInches * COUNTS_PER_INCH);
                newLeftTarget2 = LeftRear.getCurrentPosition() - (int) (strafeInches * COUNTS_PER_INCH);
                newRightTarget2 = RightRear.getCurrentPosition() + (int) (strafeInches * COUNTS_PER_INCH);

                LeftFront.setTargetPosition(newLeftTarget);
                RightFront.setTargetPosition(newRightTarget);
                LeftRear.setTargetPosition(newLeftTarget2);
                RightRear.setTargetPosition(newRightTarget2);

                // Turn On RUN_TO_POSITION
                LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (LeftFront.isBusy() && RightFront.isBusy() && LeftRear.isBusy() && RightRear.isBusy())) {
                    double YawAngle = orientation.getYaw(AngleUnit.DEGREES);
                    if (YawAngle < (DesiredAngle - .5)) {
                        //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                        //A negative YawAngle is a clockwise rotation for the robot.
                        LeftFront.setPower(speed);
                        RightFront.setPower(speed);
                        LeftRear.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        RightRear.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                    } else if (YawAngle > (DesiredAngle + .5)) {
                        //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                        //A positive YawAngle is a counter-clockwise rotation for the robot.
                        LeftFront.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        RightFront.setPower(speed - ((abs(DesiredAngle - YawAngle)) / 75));
                        LeftRear.setPower(speed);
                        RightRear.setPower(speed);
                    } else {
                        LeftFront.setPower(speed);
                        RightFront.setPower(speed);
                        LeftRear.setPower(speed);
                        RightRear.setPower(speed);
                    }
                    telemetry.addData("Angle is", YawAngle);
                    telemetry.addData("Motor Power is", "At %7f :%7f :%7f :%7f", LeftFront.getPower(), RightFront.getPower(), LeftRear.getPower(), RightRear.getPower());
                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftTarget2, newRightTarget2);
                    telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            LeftFront.getCurrentPosition(),
                            RightFront.getCurrentPosition(),
                            LeftRear.getCurrentPosition(),
                            RightRear.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                sleep(50);
                LeftFront.setPower(0);
                RightFront.setPower(0);
                LeftRear.setPower(0);
                RightRear.setPower(0);

                // Turn off RUN_TO_POSITION
                LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        }
        public void IMUTurn ( double DesiredAngle, double timeoutS){
            if (opModeIsActive()) {
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < timeoutS) {
                    double YawAngle = orientation.getYaw(AngleUnit.DEGREES);
                    if (YawAngle < DesiredAngle - 0.5) {
                        LeftFront.setPower(-((abs(DesiredAngle - YawAngle)) / 75));
                        RightFront.setPower(((abs(DesiredAngle - YawAngle)) / 75));
                        LeftRear.setPower(-((abs(DesiredAngle - YawAngle)) / 75));
                        RightRear.setPower(((abs(DesiredAngle - YawAngle)) / 75));
                    } else if (YawAngle > DesiredAngle + 0.5) {
                        LeftFront.setPower(((abs(DesiredAngle - YawAngle)) / 75));
                        RightFront.setPower(-((abs(DesiredAngle - YawAngle)) / 75));
                        LeftRear.setPower(((abs(DesiredAngle - YawAngle)) / 75));
                        RightRear.setPower(-((abs(DesiredAngle - YawAngle)) / 75));
                    } else {
                        LeftFront.setPower(0);
                        LeftRear.setPower(0);
                        RightFront.setPower(0);
                        RightRear.setPower(0);
                    }
                    telemetry.addData("Angle is", YawAngle);
                    telemetry.addData("Motor Power is", "At %7f :%7f :%7f :%7f", LeftFront.getPower(), RightFront.getPower(), LeftRear.getPower(), RightRear.getPower());
                    telemetry.update();
                }
                //sleep(50);
                LeftFront.setPower(0);
                RightFront.setPower(0);
                LeftRear.setPower(0);
                RightRear.setPower(0);
            }
        }


}   // end class
