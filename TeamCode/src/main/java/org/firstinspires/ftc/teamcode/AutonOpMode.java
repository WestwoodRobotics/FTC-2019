package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Drive Encoder", group="Exercises")
public class AutonOpMode extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

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

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
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

        leftMotor.setPower(-1);
        rightMotor.setPower(1);

        leftMotor.setTargetPosition(desiredTick);
        rightMotor.setTargetPosition(desiredTick);


        // set left motor to run to target encoder position and stop with brakes on.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set right motor to run without regard to an encoder.


        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to 5000 encoder counts.


        // set both motors to 25% power. Movement will start.
        while (Math.abs(leftMotor.getCurrentPosition()) < desiredTick)
        {
            telemetry.addData("Position Left: ", leftMotor.getCurrentPosition());
            telemetry.addData("Position Right: ", rightMotor.getCurrentPosition());

            telemetry.update();
        }

        rightMotor.setPower(0);


    }

}

