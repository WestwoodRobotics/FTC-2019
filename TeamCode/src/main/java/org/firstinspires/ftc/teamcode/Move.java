package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Move{

    private double degrees;
    private String turnDirection;
    private double turnTicks;
    private static final double DIAMETER = 14;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor centerMotor;
    private int requiredTicks;
    private double inches;
    private ColorSensor colorSensorOne;
    private ColorSensor colorSensorTwo;
    private static final double BLOCK_TO_MID = 3;
    private static final double BLOCK_lENGTH = 3;
    private int secondBlock;

    private Telemetry telemetry;

    public Move(DcMotor leftMotor, DcMotor rightMotor, DcMotor centerMotor, Telemetry telemetry){
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.centerMotor = centerMotor;
        this.telemetry = telemetry;
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
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setTargetPosition(requiredTicks);
        rightMotor.setTargetPosition(requiredTicks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(-1);
        rightMotor.setPower(1);

        while(leftMotor.isBusy() || rightMotor.isBusy()){
            telemetry.addData("encoder-right", rightMotor.getCurrentPosition());
            telemetry.update();

            telemetry.addData("target-position", rightMotor.getTargetPosition());
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void turn(String turnDirection, double degrees) {
        this.turnDirection = turnDirection;
        this.degrees = degrees;

        if(turnDirection.equals("LEFT")){
            leftMotor.setPower(1);
            rightMotor.setPower(1);

            degrees = degrees % 180;
        }
        else if(turnDirection.equals("RIGHT")){
            leftMotor.setPower(-1);
            rightMotor.setPower(-1);

            degrees = Math.abs(0 - degrees) % 180;
        }

        turnTicks= DIAMETER*degrees;

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setTargetPosition((int) Math.round(turnTicks));
        rightMotor.setTargetPosition((int) Math.round(turnTicks));

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void turnAround(){
        leftMotor.setPower(1);
        rightMotor.setPower(1);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setTargetPosition((int)DIAMETER*180);
        rightMotor.setTargetPosition((int)DIAMETER*180);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public boolean isTape(){
        if((colorSensorOne.blue() >= 150) || (colorSensorOne.red() >= 150)) {
            return true;
        }

        return false;
    }

    public boolean isCapStone(){
        if((colorSensorTwo.red() <= 100)){
            return true;
        }
        return false;
    }

    public void checkStones(){
        boolean found = false;
        while(found == false) {
            this.goForward(BLOCK_lENGTH);
            found = this.isCapStone();
            secondBlock++;
        }
    }

    public int getSecondBlock(){
        return secondBlock;
    }

    public double getDegrees(){
        return degrees;
    }

    public String getTurnDirection(){
        return turnDirection;
    }

    public double getInches(){
        return inches;
    }
}