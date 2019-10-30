package org.firstinspires.ftc.teamcode;

import java.nio.channels.InterruptedByTimeoutException;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
public class TeleopOpMode  extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centerDrive = null;

    private DcMotor leftIntakeDrive = null;
    private DcMotor rightIntakeDrive = null;

    private Servo intakeServo = null;

    boolean buttonState=true;
    boolean numButton=true;

    String intakeMotorsOn = "";
    String intakeOpen = "";

    double leftPower;
    double rightPower;
    double centerPower;

    double intakeServoPower;

    double intakeFunctionTime = 3;

    double leftIntakePower;
    double rightIntakePower;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        centerDrive = hardwareMap.get(DcMotor.class, "centerDrive");

        leftIntakeDrive = hardwareMap.get(DcMotor.class, "leftIntakeDrive");
        rightIntakeDrive = hardwareMap.get(DcMotor.class, "leftIntakeDrive");

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        centerDrive.setDirection(DcMotor.Direction.REVERSE);

        leftIntakeDrive.setDirection(DcMotor.Direction.FORWARD);
        rightIntakeDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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


    public void sleep()throws InterruptedException{
        TimeUnit.SECONDS.sleep(1);
    }

    @Override
    public void loop(){
        // Setup a variable for each drive wheel to save power level for telemetry




        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.
        //
        //        // POV Mode uses left stick to go forward, and right stick to turn.
        //        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;

        //Checking if A button is pressed and changing hold to click
        if (gamepad1.a){
            if (numButton) {
                buttonState = !buttonState;
                numButton=false;
            }
        }

        else {
            numButton=true;
        }

        if(gamepad2.a){
            leftIntakePower = 1;
            rightIntakePower = 1;
            intakeMotorsOn = "ON";
            intakeOpen = "OPEN";

            leftIntakeDrive.setPower(leftIntakePower);
            rightIntakeDrive.setPower(rightIntakePower);
        }

        else {
            intakeMotorsOn = "OFF";
            intakeOpen = "CLOSED";

            leftIntakePower = 0;
            rightIntakePower= 0;
        }

        if (gamepad2.x && !(intakeServo.getPosition() >= intakeServo.MAX_POSITION)) {
            intakeServo.setPosition(intakeServo.getPosition() + (double)1/180);
        }

        else if (gamepad2.y && !gamepad2.x && !(intakeServo.getPosition() >= intakeServo.MIN_POSITION)) {
            intakeServo.setPosition(intakeServo.getPosition() - (double)1/180);
        }

        if(buttonState){
            //Tank Drive
            leftPower  = gamepad1.left_stick_y ;
            rightPower = gamepad1.right_stick_y ;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            centerDrive.setPower(0);
        }

        else{
            //Horizontal Movement
            centerPower = gamepad1.right_stick_x ;
            centerDrive.setPower(centerPower);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }



        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Power", "Horizontal: (%.2f)", centerPower);
        telemetry.addData("Drive Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        telemetry.addData("Intake Motors", intakeMotorsOn);
        telemetry.addData("Intake Open", intakeOpen);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
