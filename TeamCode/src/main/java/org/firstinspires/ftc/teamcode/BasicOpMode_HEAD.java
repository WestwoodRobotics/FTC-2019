package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Test")
public class BasicOpMode_HEAD extends OpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor bottomleftDrive = null;
    private DcMotor bottomrightDrive = null;
    private DcMotor topleftDrive = null;
    private DcMotor toprightDrive = null;

    @Override
    public void init(){
        bottomleftDrive = hardwareMap.get(DcMotor.class, "bottom_left_drive");
        bottomrightDrive = hardwareMap.get(DcMotor.class, "bottom_right_drive");
        topleftDrive = hardwareMap.get(DcMotor.class, "top_left_drive");
        toprightDrive = hardwareMap.get(DcMotor.class, "top_right_drive");

        bottomleftDrive.setDirection(DcMotor.Direction.FORWARD);
        bottomrightDrive.setDirection(DcMotor.Direction.REVERSE);
        topleftDrive.setDirection(DcMotor.Direction.FORWARD);
        toprightDrive.setDirection(DcMotor.Direction.REVERSE);

        bottomleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        toprightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        bottomleftDrive.setPower(.5d);
        bottomrightDrive.setPower(.5d);
        topleftDrive.setPower(.5d);
        toprightDrive.setPower(.5d);

        telemetry.addData("bottom left", ": " + bottomleftDrive.getPower());
        telemetry.addData("bottom right", ": " + bottomleftDrive.getPower());
        telemetry.addData("top left", ": " + bottomleftDrive.getPower());
        telemetry.addData("top right", ": " + bottomleftDrive.getPower());

    }

    @Override
    public void stop(){

    }
}
