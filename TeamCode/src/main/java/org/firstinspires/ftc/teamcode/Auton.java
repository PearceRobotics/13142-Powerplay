package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import  com.qualcomm.robotcore.hardware.TouchSensor;
public class Auton extends motorsetup
    {
    double clicksPerInch = 8.1;

    int leftFrontPos = leftFrontDrive.getCurrentPosition();
    int rightFrontPos = rightFrontDrive.getCurrentPosition();
    int leftBackPos = leftBackDrive.getCurrentPosition();
    int rightBackPos = rightBackDrive.getCurrentPosition();

        public void forward(double howFar, double howFast)
        {
            leftFrontPos += howFar * clicksPerInch;
            rightFrontPos += howFar * clicksPerInch;
            leftBackPos += howFar * clicksPerInch;
            rightBackPos += howFar * clicksPerInch;

            leftFrontDrive.setPower(howFast);
            rightFrontDrive.setPower(howFar);
            leftBackDrive.setPower(howFar);
            rightBackDrive.setPower(howFar);

            leftFrontDrive.setTargetPosition(leftFrontPos);
            rightFrontDrive.setTargetPosition(rightFrontPos);
            leftBackDrive.setTargetPosition(leftBackPos);
            rightBackDrive.setTargetPosition(rightBackPos);
        }

    }
