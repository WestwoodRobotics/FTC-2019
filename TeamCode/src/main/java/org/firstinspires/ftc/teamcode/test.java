package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class test extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Turn turner = new Turn();
        Move mover = new Move();
        leftMotor = hardwareMap.dcMotor.get("leftDrive");
        rightMotor = hardwareMap.dcMotor.get("rightDrive");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        mover.goForward(12);
        turner.turn("LEFT", 90);
        mover.goForward(12);
        turner.turn("RIGHT", 30);
        mover.goForward(14);
        turner.turnAround();

        telemetry.addData("Forward Motion", mover.getInches());
        telemetry.addData("Turn", turner.getDegrees());
        telemetry.addData("Turn", turner.getDirection());

    }
}
