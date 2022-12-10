/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

@Autonomous
public class apriltags extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

     DcMotor leftFrontDrive = null;
     DcMotor rightFrontDrive = null;
     DcMotor leftBackDrive = null;
     DcMotor rightBackDrive = null;
     DcMotor leftMotor = null;
     DcMotor rightMotor = null;
     DcMotor intakeMotor = null;

    int leftFrontPos, rightFrontPos, leftBackPos, rightBackPos, leftPos, rightPos;

    static double clicksPerInch = 56.989;
    static double clicksPerArmInch = 205.729;


    String parking = "";

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_left_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        leftBackDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //arm motor
        leftMotor = hardwareMap.get(DcMotor.class, "left_arm_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_arm_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");


        //arm modes
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        telemetry.setMsTransmissionInterval(50);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling getDetectionsUpdate() will only return an object if there was a new frame
                // processed since the last time we called it. Otherwise, it will return null. This
                // enables us to only run logic when there has been a new frame, as opposed to the
                // getLatestDetections() method which will always return an object.
                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

                // If there's been a new frame...
                if (detections != null) {
                    telemetry.addData("FPS", camera.getFps());
                    telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                    telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                    // If we don't see any tags
                    if (detections.size() == 0) {
                        numFramesWithoutDetection++;

                        // If we haven't seen a tag for a few frames, lower the decimation
                        // so we can hopefully pick one up if we're e.g. far back
                        if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                            aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                        }
                    }
                    // We do see tags!
                    else {
                        numFramesWithoutDetection = 0;

                        // If the target is within 1 meter, turn on high decimation to
                        // increase the frame rate
                        if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                            aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                        }

                        for (AprilTagDetection detection : detections) {
                            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

                        }

                    }
                    telemetry.update();
                    if (detections.size() >= 1) {
                        if (detections.get(0).id == 1) {
                            parking = "left";
                            break;
                        } else if (detections.get(0).id == 2) {
                            parking = "right";
                            break;
                        }

                    }
                }

                sleep(20);
            }
        }

        liftArm(2, 7);
        forward(30,7);

        if(parking == "left")
        {
            left(5,7);
        }
        else if(parking == "right"){
            right(5,7);
        }
    }

        private void forward(double howFar, double howFast){
            leftFrontDrive.setTargetPosition(0);
            rightFrontDrive.setTargetPosition(0);
            leftBackDrive.setTargetPosition(0);
            rightBackDrive.setTargetPosition(0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontPos = leftFrontDrive.getCurrentPosition();
            rightFrontPos = rightFrontDrive.getCurrentPosition();
            leftBackPos = leftBackDrive.getCurrentPosition();
            rightBackPos = rightBackDrive.getCurrentPosition();

            leftFrontPos += howFar * clicksPerInch;
            rightFrontPos += howFar * clicksPerInch;
            leftBackPos += howFar * clicksPerInch;
            rightBackPos += howFar * clicksPerInch;

            leftFrontDrive.setPower(howFast);
            rightFrontDrive.setPower(howFast);
            leftBackDrive.setPower(howFast);
            rightBackDrive.setPower(howFast);

            leftFrontDrive.setTargetPosition(leftFrontPos);
            rightFrontDrive.setTargetPosition(rightFrontPos);
            leftBackDrive.setTargetPosition(leftBackPos);
            rightBackDrive.setTargetPosition(rightBackPos);
        }

        public void left(double howFar, double howFast){
            leftFrontPos = leftFrontDrive.getCurrentPosition();
            rightFrontPos = rightFrontDrive.getCurrentPosition();
            leftBackPos = leftBackDrive.getCurrentPosition();
            rightBackPos = rightBackDrive.getCurrentPosition();

            leftFrontPos -= howFar * clicksPerInch;
            rightFrontPos += howFar * clicksPerInch;
            leftBackPos -= howFar * clicksPerInch;
            rightBackPos += howFar * clicksPerInch;

            leftFrontDrive.setPower(howFast);
            rightFrontDrive.setPower(howFast);
            leftBackDrive.setPower(howFast);
            rightBackDrive.setPower(howFast);

            leftFrontDrive.setTargetPosition(leftFrontPos);
            rightFrontDrive.setTargetPosition(rightFrontPos);
            leftBackDrive.setTargetPosition(leftBackPos);
            rightBackDrive.setTargetPosition(rightBackPos);
        }

        public void right(double howFar, double howFast){
            leftFrontPos = leftFrontDrive.getCurrentPosition();
            rightFrontPos = rightFrontDrive.getCurrentPosition();
            leftBackPos = leftBackDrive.getCurrentPosition();
            rightBackPos = rightBackDrive.getCurrentPosition();

            leftFrontPos += howFar * clicksPerInch;
            rightFrontPos -= howFar * clicksPerInch;
            leftBackPos += howFar * clicksPerInch;
            rightBackPos -= howFar * clicksPerInch;

            leftFrontDrive.setPower(howFast);
            rightFrontDrive.setPower(howFast);
            leftBackDrive.setPower(howFast);
            rightBackDrive.setPower(howFast);

            leftFrontDrive.setTargetPosition(leftFrontPos);
            rightFrontDrive.setTargetPosition(rightFrontPos);
            leftBackDrive.setTargetPosition(leftBackPos);
            rightBackDrive.setTargetPosition(rightBackPos);
        }

        public void liftArm ( int index, double howFast){
            int[] position = {1100, 500, 120};
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotor.setTargetPosition(0);
            rightMotor.setTargetPosition(0);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intakeMotor.setTargetPosition(0);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftPos = leftMotor.getCurrentPosition();
            rightPos = rightMotor.getCurrentPosition();

            leftMotor.setPower(howFast);
            rightMotor.setPower(howFast);

            leftMotor.setTargetPosition(position[index]);
            rightMotor.setTargetPosition(position[index]);

            leftMotor.setPower(0);
            rightMotor.setPower(0);

        }
    }







