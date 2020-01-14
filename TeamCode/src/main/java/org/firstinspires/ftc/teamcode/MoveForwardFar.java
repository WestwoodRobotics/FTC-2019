package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Move Forward Far", group = "Exercises")
public class MoveForwardFar extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor centerMotor;
    private int ticksInWheel = 580;
    private double radius = 7.5;


    public int inchesToTicks(double inches) {
        double ticksForward = inches * ticksInWheel / (4 * Math.PI);
        int theTicksForward = (int) (Math.ceil(ticksForward));
        return theTicksForward;
    }

    public double ticksToInches(int ticks) {
        double inchesForward = 4 * ticks * Math.PI / ticksInWheel;
        return inchesForward;
    }

    public int rotateTicks(float degrees) {
        int rotationTicks = inchesToTicks(degrees * radius * Math.PI / 180);
        return rotationTicks;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        double desiredInches = 25.0;

        leftMotor = hardwareMap.dcMotor.get("leftDrive");
        rightMotor = hardwareMap.dcMotor.get("rightDrive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder count kept by left motor.it


        telemetry.addData("Mode", "waiting");
        telemetry.update();
        // wait for start button.

        waitForStart();

        // GO STRAIGHT
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setPower(0.8);
        leftMotor.setPower(-1);

        rightMotor.setTargetPosition(inchesToTicks(desiredInches));
        leftMotor.setTargetPosition(inchesToTicks(desiredInches));


        // set left motor to run to target encoder position and stop with brakes on.
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set right motor to run without regard to an encoder.


        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to 5000 encoder counts.


        // set both motors to 25% power. Movement will start.
        while (Math.abs(rightMotor.getCurrentPosition()) < inchesToTicks(desiredInches)) {
            telemetry.addData("Position Center: ", Math.abs(rightMotor.getCurrentPosition()));

            telemetry.update();
        }

        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

}
