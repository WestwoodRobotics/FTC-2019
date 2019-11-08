package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Foundation and Park", group="Exercises")
public class RFoundationPark extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo backHook;


    double desiredInches = 46;

    public int inchesToTicks(double inches)
    {
        double ticksForward = inches*115;
        int theTicksForward = (int)(Math.ceil(ticksForward));
        return theTicksForward;
    }
    public double ticksToInches(int ticks)
    {
        double inchesForward = ticks/115.0;
        return inchesForward;
    }

    public double ticksToDegrees(int ticks)
    {
        double inchesForward=ticksToInches(ticks);
        double circleFraction = inchesForward/14 *Math.PI;
        double degreesForward = circleFraction * 360;
        return degreesForward;
    }
    public int degreesToDistance(double degrees)
    {
        double rotation = degrees/360;
        double rotationDist = rotation * 14 *Math.PI;
        int ticks = inchesToTicks(rotationDist);
        return ticks;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("leftDrive");
        rightMotor = hardwareMap.dcMotor.get("rightDrive");
        backHook= hardwareMap.servo.get("backHooks");

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

        leftMotor.setPower(1);
        rightMotor.setPower(-1);

        leftMotor.setTargetPosition(inchesToTicks(desiredInches));
        rightMotor.setTargetPosition(inchesToTicks(desiredInches));


        // set left motor to run to target encoder position and stop with brakes on.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set right motor to run without regard to an encoder.


        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to 5000 encoder counts.


        // set both motors to 25% power. Movement will start.

        while (Math.abs(leftMotor.getCurrentPosition()) > inchesToTicks(desiredInches))
        {
            telemetry.addData("Position Left: ", ticksToInches(leftMotor.getCurrentPosition()));
            telemetry.addData("Position Right: ", ticksToInches(rightMotor.getCurrentPosition()));

            telemetry.update();
        }

        backHook.setPosition(0.5);

        desiredInches=46;

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(-1);
        rightMotor.setPower(1);

        leftMotor.setTargetPosition(inchesToTicks(desiredInches));
        rightMotor.setTargetPosition(inchesToTicks(desiredInches));


        // set left motor to run to target encoder position and stop with brakes on.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set right motor to run without regard to an encoder.


        // set left motor to run to 5000 encoder counts.


        // set both motors to 25% power. Movement will start.

        while (Math.abs(leftMotor.getCurrentPosition()) < inchesToTicks(desiredInches))
        {
            telemetry.addData("Position Left: ", ticksToInches(leftMotor.getCurrentPosition()));
            telemetry.addData("Position Right: ", ticksToInches(rightMotor.getCurrentPosition()));

            telemetry.update();
        }



    }

}
