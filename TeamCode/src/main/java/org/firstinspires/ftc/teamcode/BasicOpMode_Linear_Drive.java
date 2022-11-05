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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name = "Basic: Linear OpMode", group = "Robot code")
public class BasicOpMode_Linear_Drive extends motorsetup {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        super.runOpMode();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Waitforthegametostart(driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match(driver presses STOP)

        while (opModeIsActive()) {

            double backRightPower = 0;
            double frontRightPower = 0;
            double backLeftPower = 0;
            double frontLeftPower = 0;
            //double intakePower = 0;
            double r = -gamepad1.left_stick_y; // Remember, this is reversed!
            //double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double sr = gamepad1.right_trigger;
            double sl = -gamepad1.left_trigger;
            double ro = -gamepad1.right_stick_x;


            boolean auhj = gamepad1.y;
            boolean imu = gamepad1.right_bumper;
            boolean imd = gamepad1.left_bumper;
            boolean aumj = gamepad1.a;

//
            //  Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that 's not used.  The default below is POV.
            // Denominator is the largest motor power (absolute value)or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the[] range -1, 1]
            /*
            double denominator = Math.max(Math.abs(y)  + Math.abs(sl), 1);
            double frontLeftPower = (y  + sl + sr) / denominator;
            double backLeftPower = (y + sl + sr) / denominator;
            double frontRightPower = (y - sl -sr) / denominator;
            double backRightPower = (y - sl - sr) / denominator;
            */


            if (r > .1 || r < -.1) {
                frontLeftPower = r;
                backLeftPower = r;
                frontRightPower = r;
                backRightPower = r;
            }

            if (sl < -.1 || sr > .1) {
                frontLeftPower -= sl + sr;
                backLeftPower -= sl + sr;
                frontRightPower += sl + sr;
                backRightPower += sl + sr;
            }

            if (ro < -.1 || ro > .1) {
                frontLeftPower -= ro;
                backLeftPower += ro;
                frontRightPower += ro;
                backRightPower -= ro;
            }


            if (gamepad1.y) {
                leftMotor.setPower(.3);
                rightMotor.setPower(.3);
                //change the setPosition
                int setPosition = 200;
                leftMotor.setTargetPosition(setPosition);
                rightMotor.setTargetPosition(setPosition);

            }
            
            // WE WILL HAVE TO REDO THE CLICKS FOR THE SETPOSITIONS SINCE WE CHANGED THE GEAR RATIOS

            if (gamepad1.b) {
                leftMotor.setPower(.3);
                rightMotor.setPower(.3);
                // change the setPosition
                int setPosition = 200;
                leftMotor.setTargetPosition(setPosition);
                rightMotor.setTargetPosition(setPosition);

            }

            if(gamepad1.a){
                leftMotor.setPower(.1);
                rightMotor.setPower(.1);
                int setPosition = 100;
                leftMotor.setTargetPosition(setPosition);
                rightMotor.setTargetPosition(setPosition);

            }
            if(gamepad1.right_bumper){
                leftMotor.setPower(.1);
                rightMotor.setPower(.1);
                int setPosition = 100;
                leftMotor.setTargetPosition(setPosition);
                rightMotor.setTargetPosition(setPosition);

            }
            /*
            if (imu) {
                intakePower = 1.0;
            }

            if(imd){
                intakePower = -1.0;
            }

             */
            // Send calculated power to wheels
            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);
            //intakeMotor.setPower(intakePower);

            // FOR THE ARM POSITIONS WE CAN USE THE GET POSITION DATA TO ADD OR SUBTRACT TO THE CURRENT
            // POSITION TO GET TO THE TARGET POSITION SO THAT WE DON'T HAVE TO RETURN TO THE BOTTOM EACH TIME
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor level l", "Left Arm Position: " + leftMotor.getCurrentPosition());
            telemetry.addData("motor level r", "Right Arm Position: " + rightMotor.getCurrentPosition());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);
            telemetry.addData("r value: ", "(%.2f)", r);
            telemetry.addData("sl value: ", "(%.2f)", sl);
            telemetry.addData("sr value: ", "(%.2f)", sr);
            telemetry.addData("ro value: ", "(%.2f)", ro);
            telemetry.addData("frontLeftDrive", "frontLeftDrive: " + leftFrontDrive.getCurrentPosition());
            telemetry.addData("frontRightDrive", "frontRightDrive: " + rightFrontDrive.getCurrentPosition());
            telemetry.addData("backLeftDrive", "backLeftDrive: " + leftBackDrive.getCurrentPosition());
            telemetry.addData("backRightDrive", "backRightDrive: " + rightBackDrive.getCurrentPosition());
            telemetry.update();
        }


        // frontLeftPower = frontLeftPower + sl;


        //Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and
        //keep straight.
        // leftPower = -gamepad1.left_stick_y;
        //  rightPower = -gamepad1.right_stick_y;

        // Show the elapsed game time and wheel power.
    }
    }
