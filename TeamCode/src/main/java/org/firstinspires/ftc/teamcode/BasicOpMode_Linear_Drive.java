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

        int armPosition = 0;
        double power = 0.5;
        while (opModeIsActive()) {

            double backRightPower = 0;
            double frontRightPower = 0;
            double backLeftPower = 0;
            double frontLeftPower = 0;
            double intakePower = 0;
            double fla = 1.1;
            double bla = 1.1;
            double bra = .90;
            double r = -gamepad1.left_stick_y; // Remember, this is reversed!
            //double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double sr = gamepad1.right_trigger;
            double sl = -gamepad1.left_trigger;
            double ro = -gamepad1.right_stick_x;

            boolean imu = gamepad1.right_bumper;
            boolean imd = gamepad1.left_bumper;

            //higher lower
            boolean higher;
            double distance = (armPosition - (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition())/2);


            if (r > .1 || r < -.1) {
                frontLeftPower = r * fla;
                backLeftPower = r * bla;
                frontRightPower = r *fla;
                backRightPower = r * bra;
            }

            if (sl < -.1 || sr > .1) {
                frontLeftPower -= (sl + sr) * fla;
                backLeftPower -= (sl + sr) * bla;
                frontRightPower += (sl + sr) * fla;
                backRightPower += (sl + sr) * bra;
            }

            if (ro < -.1 || ro > .1) {
                frontLeftPower -= ro * fla;
                backLeftPower += ro * bla;
                frontRightPower += ro * fla;
                backRightPower -= ro * bra;
            }

            if(distance > 0){
                higher = true;
                }
            else{
                higher = false;
            }

            int[] position = {1100, 500, 150};

            if (gamepad1.y && higher) {
                leftMotor.setPower(power);
                rightMotor.setPower(power);
                armPosition = position[0];

                telemetry.addData("test", "is this working");
            }
            else if (gamepad1.y && !higher){
                power = .3;
                leftMotor.setPower(power);
                rightMotor.setPower(power);
                armPosition = position[0];
            }

            // WE WILL HAVE TO REDO THE CLICKS FOR THE SETPOSITIONS SINCE WE CHANGED THE GEAR RATIOS

            if (gamepad1.b && higher) {
                leftMotor.setPower(power);
                rightMotor.setPower(power);
                armPosition = position[1];
            }
            else if (gamepad1.b && !higher){
                power = .3;
                leftMotor.setPower(power);
                rightMotor.setPower(power);
                armPosition = position[1];

            }

            if(gamepad1.a && higher){
                leftMotor.setPower(power);
                rightMotor.setPower(power);

                armPosition = position[2];
            }
            else if (gamepad1.a && !higher){
                power = .3;
                leftMotor.setPower(power);
                rightMotor.setPower(power);
                armPosition = position[2];
            }

            leftMotor.setTargetPosition(armPosition);
            rightMotor.setTargetPosition(armPosition);

            if (imu) {
                intakeMotor.setPower(.75);
            }

            if (imd){
                intakeMotor.setPower(-.75);
            }


/*
            if (magnet.isPressed()){
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
*/
            //maybe use setposition to get to the different junctions
            //and use setpower to pick up cones
            /*
            if(gamepad1.dpad_up){
                leftMotor.setPower(power);
                rightMotor.setPower(power);
                int setLeftPosition = leftMotor.getCurrentPosition()+20;
                int setRightPosition = rightMotor.getCurrentPosition()+20;
                leftMotor.setTargetPosition(setLeftPosition);
                rightMotor.setTargetPosition(setRightPosition);
            }

            if(gamepad1.dpad_down){
                leftMotor.setPower(power);
                rightMotor.setPower(power);
                int setLeftPosition = leftMotor.getCurrentPosition()-20;
                int setRightPosition = rightMotor.getCurrentPosition()-20;
                leftMotor.setTargetPosition(setLeftPosition);
                rightMotor.setTargetPosition(setRightPosition);
            }

            */

            //I might have to create a reset button for the arm motors as the tilt may also be a
            //Programming issue stemming from the encoders

            //That does not mean that it can't be fixed mechanically

            //resetting the encoders may remove any of the errors that skipping cause

            // Send calculated power to wheels
            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);
            intakeMotor.setPower(intakePower);


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
            telemetry.addData("intake", "intake: " + intakeMotor.getCurrentPosition());
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
