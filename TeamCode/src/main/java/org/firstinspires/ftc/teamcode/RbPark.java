package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="RbPark", group="Exercises")
public class RbPark extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor centerMotor;

    int desiredTick = 1440;
    double desiredInches = 12.0;

    public int inchesToTicks(double inches)
    {
        double ticksForward = inches*Math.PI;
        int theTicksForward = (int)(Math.ceil(ticksForward));
        return theTicksForward;
    }
    public double ticksToInches(int ticks)
    {
        double inchesForward = ticks/Math.PI;
        return inchesForward;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("leftDrive");
        rightMotor = hardwareMap.dcMotor.get("rightDrive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // reset encoder count kept by left motor.


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setPower(1);
        leftMotor.setPower(1);

        rightMotor.setTargetPosition(desiredTick);
        leftMotor.setTargetPosition(desiredTick);


        // set left motor to run to target encoder position and stop with brakes on.
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set right motor to run without regard to an encoder.


        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to 5000 encoder counts.


        // set both motors to 25% power. Movement will start.
        while (Math.abs(rightMotor.getCurrentPosition()) < desiredTick)
        {
            telemetry.addData("Position Center: ", Math.abs(rightMotor.getCurrentPosition()));

            telemetry.update();
        }

    /*  while(true){
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }*/

        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

}

