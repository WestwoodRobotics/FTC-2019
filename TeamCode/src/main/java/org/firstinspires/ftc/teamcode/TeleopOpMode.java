package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
public class TeleopOpMode  extends OpMode {
    // Declare OpMode members.

    //time variable object initialized
    private ElapsedTime runtime = new ElapsedTime();

    //drive motors initialized
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centerDrive = null;

    //back Servos
    private Servo leftBack;
    private Servo rightBack;


    //elevator motors initialized
    private DcMotor leftElevator = null;
    private DcMotor rightElevator = null;

    //intake motors initialized
    private DcMotor runIntakeOpen = null;
    private DcMotor leftIntakeWheel = null;
    private DcMotor rightIntakeWheel = null;



    //speed and button variables
    Double SLOWNESS = 0.4;
    Double SPEED_MULTIPLIER = 1.25;
    Double IntakeSlow = .6;

    boolean buttonStateSlow=true;
    boolean numButtonSlow=true;
    boolean leftTriggerPress = false;
    boolean rightTriggerPress = false;

    //telemetry variables
    String intakeMotorsOn = "";
    String intakeOpen = "";

    //power variables
    double leftPower;
    double rightPower;
    double centerPower;

    double rightElevatorPower;
    double leftElevatorPower;

    boolean IntakeToggle;

    double runIntakePower;
    double rightIntakeWheelPower;
    double leftIntakeWheelPower;

    /*
     * Code to run ONCE when t+he driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializxed");

        //initialize motors for phone
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        centerDrive = hardwareMap.get(DcMotor.class, "centerDrive");

        leftIntakeWheel = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntakeWheel = hardwareMap.get(DcMotor.class, "rightIntake");
        runIntakeOpen = hardwareMap.get(DcMotor.class, "runIntake");

        leftElevator = hardwareMap.get(DcMotor.class, "leftElevator");
        rightElevator = hardwareMap.get(DcMotor.class, "rightElevator");

        leftBack = hardwareMap.servo.get("leftBack");
        rightBack = hardwareMap.servo.get("rightBack");

        //sets direction of motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        centerDrive.setDirection(DcMotor.Direction.REVERSE);

        leftIntakeWheel.setDirection(DcMotor.Direction.REVERSE);
        rightIntakeWheel.setDirection(DcMotor.Direction.FORWARD);


        leftElevator.setDirection(DcMotor.Direction.REVERSE);
        rightElevator.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override

    //Drive Controlls setup

    public void loop(){
        //slow mode
        if (gamepad1.b){
            if (numButtonSlow) {
                buttonStateSlow= !buttonStateSlow;
                numButtonSlow=false;
            }
        }

        else {
            numButtonSlow=true;
        }
        //Back Hook
        if(gamepad2.y){
            leftBack.setPosition(180);
            rightBack.setPosition(0);
        }
        else if(gamepad2.a){
            leftBack.setPosition(0);
            rightBack.setPosition(180);
        }

        //open intake
        if (gamepad2.left_trigger != 0 && gamepad2.right_trigger == 0){
            leftTriggerPress = true;
            rightTriggerPress = false;
        }
        else if (gamepad2.right_trigger != 0 && gamepad2.left_trigger == 0 ){
            rightTriggerPress = true;
            leftTriggerPress = false;
        }
        else if (gamepad2.right_trigger != 0 && gamepad2.left_trigger != 0){
            leftTriggerPress = false;
            rightTriggerPress = false;
        }

        /////////////
        if (rightTriggerPress && !leftTriggerPress) {
            runIntakePower = 1;

        }
        else if (leftTriggerPress && !rightTriggerPress){
            runIntakePower = -1;

        }
        else if (!rightTriggerPress && !leftTriggerPress){
            runIntakePower = 0  ;

        }
        else{
            runIntakePower = 0;
        }

        //run intake wheels
        if (gamepad2.right_stick_y == 0){
            leftIntakeWheelPower = 0;
            rightIntakeWheelPower = 0;
        }
        else if(gamepad2.right_stick_y > 0){
            leftIntakeWheelPower = gamepad2.right_stick_y;
            rightIntakeWheelPower = gamepad2.right_stick_y;
        }
        else if(gamepad2.right_stick_y < 0) {
            leftIntakeWheelPower = -1;
            rightIntakeWheelPower = -1;
        }


        //elevator code
        if(gamepad2.dpad_down && gamepad2.dpad_up){
            rightElevatorPower = 0;
            leftElevatorPower = 0;
        }
        else if (gamepad2.dpad_up){
            rightElevatorPower = 1;
            leftElevatorPower = 1;
        }
        else if (gamepad2.dpad_down){
            rightElevatorPower = -1;
            leftElevatorPower = -1;
        }
        else {
            rightElevatorPower = 0;
            leftElevatorPower = 0;
        }


        //center wheel
        if (gamepad1.left_trigger>0 &&  gamepad1.right_trigger>0){
            centerPower=0;
        }
        else if (gamepad1.left_trigger>0) {
            centerPower = -gamepad1.left_trigger;

        }else if (gamepad1.right_trigger>0){
            centerPower=gamepad1.right_trigger;
        }else{
            centerPower=0;
        }
        // Mode for Intake



        //applies slowness variable
        if (buttonStateSlow){
            centerPower=centerPower*SLOWNESS*2;
            leftPower  = gamepad1.left_stick_y * SLOWNESS;
            rightPower = gamepad1.right_stick_y * SLOWNESS ;

        }else{
            leftPower  = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;


        }// Tank Drive





        //set powers of motors

        leftIntakeWheel.setPower(rightIntakeWheelPower);
        rightIntakeWheel.setPower(leftIntakeWheelPower);
        runIntakeOpen.setPower(runIntakePower*IntakeSlow);
        centerDrive.setPower(centerPower);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        leftElevator.setPower(leftElevatorPower);
        rightElevator.setPower(rightElevatorPower);

        telemetry.addData("Slow Mode: ", " " + buttonStateSlow);
        telemetry.addData("Right Elevator: ", " " + rightElevatorPower);
        telemetry.addData("Left Elevator: " , " " + leftElevatorPower);
        telemetry.addData(
                "Status", "Run Time: " + runtime.toString());
        telemetry.addData("Power", "Horizontal: (%.2f)", centerPower);
        telemetry.addData("Drive Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        telemetry.addData("Intake Motors ", intakeMotorsOn);
        telemetry.addData("Intake Open ", intakeOpen);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
