package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Turn {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private double degrees;
    private String turnDirection;
    private double turnTicks;
    private static final double DIAMETER = 14;
    public Turn() {

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
    }

    public double getDegrees(){
        return degrees;
    }

    public String getDirection(){
        return turnDirection;
    }
}

