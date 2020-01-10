
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Left Foundation and Park", group="Exercises")
public class LFoundationPark extends LinearOpMode {
   DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor centerMotor;



    double desiredInches = 46;

    public int inchesToTicks(double inches)
    {
        double ticksForward = inches*(360*Math.PI);
        int theTicksForward = (int)(Math.ceil(ticksForward));
        return theTicksForward;
    }
    public double ticksToInches(int ticks)
    {
        double inchesForward = ticks/Math.PI;
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
        centerMotor = hardwareMap.dcMotor.get("centerDrive");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        centerMotor.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder count kept by left motor.


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        if(Math.abs(leftMotor.getCurrentPosition()) < 100){
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(0.2);
        rightMotor.setPower(-0.2);
        if(Math.abs(leftMotor.getCurrentPosition()) > 100){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        leftMotor.setTargetPosition(100);
        rightMotor.setTargetPosition(100);


        // set left motor to run to target encoder position and stop with brakes on.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set left motor to run to 5000 encoder counts.


        // set both motors to 25% power. Movement will start.

        while (Math.abs(leftMotor.getCurrentPosition()) < 100)
        {
            telemetry.addData("Position Left: ", leftMotor.getCurrentPosition());
            telemetry.addData("Position Right: ", rightMotor.getCurrentPosition());
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

}