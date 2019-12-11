package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="test", group="Exercises")
public class test extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor centerMotor;

    @Override
    public void runOpMode()
    {
        leftMotor = hardwareMap.dcMotor.get("leftDrive");
        rightMotor = hardwareMap.dcMotor.get("rightDrive");
        centerMotor = hardwareMap.dcMotor.get("centerDrive");
        Move mover = new Move(leftMotor, rightMotor, centerMotor, telemetry);


        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //mover.goForward(12,1);
        //mover.goBackward(12,1);
        mover.turn("LEFT",90,1);
        mover.goForward(12,1);
        mover.turn("RIGHT",90,1);
        mover.turnAround();

       // mover.goForward(12,10);
       // mover.goBackward(12,10);
       // mover.goLeft(12,10);
       // mover.goRight(12,10);
       // mover.turn("LEFT",90,10);
       // mover.goForward(12,10);
       // mover.turn("RIGHT",90,10);

     }


}
