package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Move Forward", group = "Exercises")
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
        double desiredInches = 2.0;

        leftMotor = hardwareMap.dcMotor.get("leftDrive");
        rightMotor = hardwareMap.dcMotor.get("rightDrive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder count kept by left motor.it


        telemetry.addData("Mode", "waiting");
        telemetry.update();
        // wait for start button.

        waitForStart();

        AutonCommand.moveForward(leftMotor,rightMotor,18, telemetry);

        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

}
