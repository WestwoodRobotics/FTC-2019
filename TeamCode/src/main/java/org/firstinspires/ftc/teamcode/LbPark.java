package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="LbPark", group="Exercises")
public class LbPark extends LinearOpMode {

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
        centerMotor = hardwareMap.dcMotor.get("centerDrive");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        centerMotor.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder count kept by left motor.


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        centerMotor.setPower(1);

        centerMotor.setTargetPosition(desiredTick);


        // set left motor to run to target encoder position and stop with brakes on.
        centerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set right motor to run without regard to an encoder.


        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to 5000 encoder counts.


        // set both motors to 25% power. Movement will start.
        while (centerMotor.getCurrentPosition() < desiredTick)
        {
            telemetry.addData("Position Center: ", centerMotor.getCurrentPosition());

            telemetry.update();
        }

        rightMotor.setPower(0);


    }

}
