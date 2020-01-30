// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.*;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="Test Parth", group="Exercises")
//@Disabled
public class TestParth extends LinearOpMode
{
    DcMotor                 leftMotor, rightMotor, centerDrive;
    //TouchSensor           touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .5, correction;
    boolean                 aButton, bButton, touched;
    private int requiredTicks;
    private double inches = 10;
    private ElapsedTime     runtime = new ElapsedTime();

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("leftDrive");
        rightMotor = hardwareMap.dcMotor.get("rightDrive");
        centerDrive = hardwareMap.dcMotor.get("centerDrive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to touch sensor.
        //      touch = hardwareMap.touchSensor.get("touch_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.



        waitForStart();
        runtime.reset();


        telemetry.addData("Mode", "running");
        telemetry.update();
        telemetry.addData("Mode","before loop");



        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // 400 ticks per motor
        /*
        int endEncodeTicks =  4000;
        int delta = 101;
        while (Math.abs(delta)> 100)
        {
            delta = endEncodeTicks - rightMotor.getCurrentPosition();
            run();
        }
        rotate(45,1);

         */
        ///while(opModeIsActive())
        //{
        //    run();
        //}
        //(2/3) of a second is 1 block


        runTime(1.5);
        rotate(90-20,.5);
        runTime(1.5);
        slideLeft(1.0);
        slideRight(1.0);

        telemetry.addData("mode","outLoop");

        rightMotor.setPower(0);
        leftMotor.setPower(0);
        stop();


        /*


        // drive until end of period.
        Calendar cal = Calendar.getInstance();
        // Use gyro to drive in a straight line.
            long startTime = cal.getTimeInMillis();
            telemetry.addData("Mode","InLoop");
            long currentTime =startTime;
            while(currentTime<startTime+10000){
                run();
                currentTime =cal.getTimeInMillis();
                if(!opModeIsActive()){
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    stop();
                }
            }

         */
        telemetry.addData("Mode","out of loop");
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        stop();
 /*

        int delta = requiredTicks;
            while (Math.abs(delta)> 100)
            {
                delta = requiredTicks - rightMotor.getCurrentPosition();
                run();
            }
            rotate(90,1);
          */


        // We record the sensor values because we will test them in more than
        // one place with time passing between those places. See the lesson on
        // Timing Considerations to know why.
        // turn the motors off.

    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    // Joshua's Code
    public int inchesToTicks(double inches)
    {
        double ticksForward = inches*360/Math.PI;
        int theTicksForward = (int)(Math.ceil(ticksForward));
        return theTicksForward;
    }
    public void slideLeft(double seconds){
        while(opModeIsActive()&& runtime.seconds()<seconds){
            centerDrive.setPower(power - correction);
        }


    }

    public void slideRight(double seconds){
        while(opModeIsActive()&& runtime.seconds()<seconds){
            centerDrive.setPower(-power + correction);
        }


    }




    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void runTime(double seconds){
        while(opModeIsActive()&& runtime.seconds()<seconds){
            run();
        }
    }
    public void run(){
        correction = checkDirection();

        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();

        leftMotor.setPower(power - correction);
        rightMotor.setPower(power + correction);
    }


}