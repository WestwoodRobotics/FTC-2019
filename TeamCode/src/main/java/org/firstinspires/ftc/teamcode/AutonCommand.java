package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonCommand {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor centerMotor;
    private static int ticksInWheel = 400;
    private static double radius = 7.5;
    static int x;



    public static int inchesToTicks(double inches) {
        double ticksForward = inches * ticksInWheel / (4 * Math.PI);
        int theTicksForward = (int) (Math.ceil(ticksForward));
        return theTicksForward;
    }

    public static double ticksToInches(int ticks) {
        double inchesForward = 4 * ticks * Math.PI / ticksInWheel;
        return inchesForward;
    }

    public static int rotateTicks(float degrees) {
        int rotationTicks = inchesToTicks(degrees * radius * Math.PI / 180);
        return rotationTicks;
    }


    public static void moveForward(DcMotor leftMotor, DcMotor rightMotor, double desiredInches,Telemetry telemetry){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setPower(-1);
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

    }
    public static void turnOnWheelLeft(DcMotor leftMotor, DcMotor rightMotor, float desiredAngle, Telemetry telemetry){
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(0);
        rightMotor.setPower(1);

        rightMotor.setTargetPosition(rotateTicks(desiredAngle));


        // set left motor to run to target encoder position and stop with brakes on.
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set right motor to run without regard to an encoder.


        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to 5000 encoder counts.


        // set both motors to 25% power. Movement will start.
        while (Math.abs(rightMotor.getCurrentPosition()) < rotateTicks(desiredAngle)) {
            telemetry.addData("Fraction of Turn: ", Math.abs(rightMotor.getCurrentPosition() / rotateTicks(desiredAngle)));
            telemetry.addData("Fraction of Turn: ", Math.abs(rightMotor.getCurrentPosition()));
            telemetry.addData("Fraction of Turn: ", rotateTicks(desiredAngle));
            telemetry.addData("f",x);
            x++;
            telemetry.update();
        }
    }
    public static void turnOnWheelRight(DcMotor leftMotor, DcMotor rightMotor, float desiredAngle, Telemetry telemetry){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(1);
        rightMotor.setPower(0);

        leftMotor.setTargetPosition(rotateTicks(desiredAngle));


        // set left motor to run to target encoder position and stop with brakes on.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set right motor to run without regard to an encoder.


        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to 5000 encoder counts.


        // set both motors to 25% power. Movement will start.
        while (Math.abs(leftMotor.getCurrentPosition()) < rotateTicks(desiredAngle)) {
            telemetry.addData("Fraction of Turn: ", Math.abs(leftMotor.getCurrentPosition()));
            telemetry.addData("Fraction of Turn: ", rotateTicks(desiredAngle));
            telemetry.addData("f",x);
            x++;
            telemetry.update();
        }
    }
    public static void stopMotors(DcMotor leftMotor, DcMotor rightMotor, Telemetry telemetry) {
        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

}
