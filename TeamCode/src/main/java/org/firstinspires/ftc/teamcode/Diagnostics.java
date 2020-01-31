/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Diagnostics")
public class Diagnostics extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor bottomleftDrive = null;
    private DcMotor bottomrightDrive = null;
    private DcMotor topleftDrive = null;
    private DcMotor toprightDrive = null;

    private DcMotor elevatorMotor = null;

    //private Servo flipperServo1 = null;
    //private Servo flipperServo2 = null;

    //private DcMotor vacuumMotor = null;

    private DcMotor elevatorMotorleft = null;
    private DcMotor elevatorMotorright = null;

    private Servo clawServo = null;
    private Servo leftArmServo = null;
    private Servo hookServo = null;
    private Servo rightArmServo = null;


    int vacuumPower = 0;

    boolean vStart = false;
    boolean vPressed = false;
    boolean vPause = false;

    boolean slowStart = false;
    boolean slowPressed = false;
    boolean slowPause = false;

    boolean cStart = false, cPressed = false, cPause = false;
    boolean hStart = false, hPressed = false, hPause = false;

    boolean INPUTstarted = false;

    boolean IOstart = false, IOpressed = false, IOpause = false;

    double eHeight = 0;

    boolean CLOSED = false, wasPressed = false;

    /*
    3 * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        bottomleftDrive = hardwareMap.get(DcMotor.class, "bottom_left_drive");
        bottomrightDrive = hardwareMap.get(DcMotor.class, "bottom_right_drive");
        topleftDrive = hardwareMap.get(DcMotor.class, "top_left_drive");
        toprightDrive = hardwareMap.get(DcMotor.class, "top_right_drive");

        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");

        //flipperServo1 = hardwareMap.get(Servo.class, "flipper_servo1");
        //flipperServo2 = hardwareMap.get(Servo.class, "flipper_servo2");

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawServo.setDirection(Servo.Direction.REVERSE);
        leftArmServo = hardwareMap.get(Servo.class, "left_arm_servo");



        hookServo = hardwareMap.get(Servo.class, "hook_servo");
        rightArmServo = hardwareMap.get(Servo.class, "right_arm_servo");
        rightArmServo.setDirection(Servo.Direction.REVERSE);

        //vacuumMotor = hardwareMap.get(DcMotor.class, "vacuum_motor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        bottomleftDrive.setDirection(DcMotor.Direction.FORWARD);
        bottomrightDrive.setDirection(DcMotor.Direction.REVERSE);
        topleftDrive.setDirection(DcMotor.Direction.FORWARD);
        toprightDrive.setDirection(DcMotor.Direction.REVERSE);


        /*elevatorMotor.setDirection(DcMotor.Direction.FORWARD);

        flipperServo1.setDirection(Servo.Direction.FORWARD);
        flipperServo2.setDirection(Servo.Direction.REVERSE);

        vacuumMotor.setDirection(DcMotor.Direction.FORWARD);*/


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
    @Override
    public void loop() {

        if (gamepad1.x) {
            hookServo.setPosition(0.5);
        } else {
            hookServo.setPosition(0);
        }
        if (gamepad1.a) {
            clawServo.setPosition(0.5);
        } else {
            clawServo.setPosition(0);
        }
        if (gamepad1.y) {
            leftArmServo.setPosition(1);
        } else {
            leftArmServo.setPosition(0);
        }
        if (gamepad1.b) {
            rightArmServo.setPosition(1);
        } else {
            rightArmServo.setPosition(0);
        }


    }

    public void stop() {
    }
}




