package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Move{

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private int requiredTicks;
    private double inches;

    public  Move() {

    }

    public int inchesToTicks(double inches)
    {
        double ticksForward = inches*360/Math.PI;
        int theTicksForward = (int)(Math.ceil(ticksForward));
        return theTicksForward;
    }

    public void goForward(double inches) {
        requiredTicks=inchesToTicks(inches);

        this.inches = inches;
        leftMotor.setPower(-1);
        rightMotor.setPower(1);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setTargetPosition(requiredTicks);
        rightMotor.setTargetPosition(requiredTicks);


    }

    public double getInches(){
        return inches;
    }
}

