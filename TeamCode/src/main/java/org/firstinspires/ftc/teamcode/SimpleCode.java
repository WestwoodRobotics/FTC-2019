package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Simple Code", group = "Exercises")
public class SimpleCode extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor centerMotor;


    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("leftDrive");
        rightMotor = hardwareMap.dcMotor.get("rightDrive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder count kept by left motor.it


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        /*
         * WRITE THE CODE HERE
         * Please note that negative parameter values will not work as intended
         * Instead, for this ability, use reversed and separate function instead
         * Angle should always be in degrees (for parameters at least)
         * Distance should always be in inches (for parameters at least)
         * */

        /*

         * MANUAL
         * moveForward(x) moves the robot forward x inches
         * turnOnWheelLeft(x) turns the robot on the point of rotation being the left wheel by x degrees
         * turnOnWheelRight(x) turns the robot on the point of rotation being the right wheel by x degrees
         *
         */

        AutonCommand.moveForward(leftMotor, rightMotor, 12, telemetry);

        AutonCommand.stopMotors(leftMotor, rightMotor, telemetry);

        AutonCommand.turnOnWheelRight(leftMotor, rightMotor, 90, telemetry);



        //Always keep these lines for safety
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

}