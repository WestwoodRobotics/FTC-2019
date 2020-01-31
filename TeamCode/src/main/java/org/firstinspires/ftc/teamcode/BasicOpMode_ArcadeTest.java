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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

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

@TeleOp(name = "Basic: Arcade Mecanum OpMode")
public class BasicOpMode_ArcadeTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor bottomleftDrive = null;
    private DcMotor bottomrightDrive = null;
    private DcMotor topleftDrive = null;
    private DcMotor toprightDrive = null;

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

    boolean slowStart = false, slowPressed = false;
    boolean xStart = false, xPressed = false;
    boolean bStart = false, bPressed = false;

    boolean INPUTstarted = false;

    boolean IOstart = false, IOpressed = false, IOpause = false;

    double eHeight = 0;
    private boolean aPressed;
    private boolean aStart;

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

        elevatorMotorright = hardwareMap.get(DcMotor.class, "elevator_motor_right");
        elevatorMotorleft = hardwareMap.get(DcMotor.class, "elevator_motor_left");

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        leftArmServo = hardwareMap.get(Servo.class, "left_arm_servo");

        clawServo.setDirection(Servo.Direction.REVERSE);

        hookServo = hardwareMap.get(Servo.class, "hook_servo");
        rightArmServo = hardwareMap.get(Servo.class, "right_arm_servo");
        rightArmServo.setDirection(Servo.Direction.REVERSE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        bottomleftDrive.setDirection(DcMotor.Direction.REVERSE);
        bottomrightDrive.setDirection(DcMotor.Direction.FORWARD);
        topleftDrive.setDirection(DcMotor.Direction.REVERSE);
        toprightDrive.setDirection(DcMotor.Direction.FORWARD);

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
        // Setup a variable for each drive wheel to save power level for telemetry
        //double bottomleftpower, bottomrightpower, topleftpower, toprightpower;
        double leftStickY = -1 * gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;

        double angle = Math.atan((leftStickY) / (leftStickX));

        // SLOW MODE


        //Tangent Inverse only goes from -pi/2 to pi/2, so I have to add some test cases to make sure
        //the angle is correct

        double[] arr = new double[4];

        if (1 + 1 == 2) {
            if (leftStickX < 0 && leftStickY < 0) { //3rd Quadrant
                angle = Math.atan((leftStickY) / (leftStickX)) + Math.PI;
            } else if (leftStickY == 0 && leftStickX < 0) {
                angle = Math.PI;
            } else if (leftStickY > 0 && leftStickX < 0) {
                angle = Math.atan((leftStickY) / (leftStickX)) + Math.PI;
            } else if (leftStickX == 0 && leftStickY > 0) {
                angle = 0.5 * Math.PI;
            } else if (leftStickX == 0 && leftStickY < 0) {
                angle = 3 * Math.PI / 2;
            } else if (leftStickX > 0 && leftStickY < 0) {
                angle += 2 * Math.PI;
            }
            angle = Math.toDegrees(angle); //Changes angle from radians to degrees

            if (angle == 0) {
                arr = new double[]{1, -1, -1, 1};
            } else if (angle > 0 && angle < 90) {
                arr = new double[]{1, angle / 45 - 1, angle / 45 - 1, 1};
            } else if (angle == 90) {
                arr = new double[]{1, 1, 1, 1};
            } else if (angle > 90 && angle < 180) {
                arr = new double[]{3 - angle / 45, 1, 1, 3 - angle / 45};
            } else if (angle == 180) {
                arr = new double[]{-1, 1, 1, -1};
            } else if (angle > 180 && angle < 270) {
                arr = new double[]{-1, 5 - angle / 45, 5 - angle / 45, -1};
            } else if (angle == 270) {
                arr = new double[]{-1, -1, -1, -1};
            } else if (angle > 270) {
                arr = new double[]{7 - angle / 45, -1, -1, 7 - angle / 45};
            }
        }




        if (gamepad2.x && !xPressed) {
            xStart = !xStart;
        }
        if (xStart) {
            rightArmServo.setPosition(1);
            leftArmServo.setPosition(1);
        }
        else{ // The issue is this if block right here
            rightArmServo.setPosition(0);
            leftArmServo.setPosition(0);
        }
        xPressed = gamepad2.x;

        if (gamepad2.b && !bPressed) {
            bStart = !bStart;
        }
        if (bStart) {
            hookServo.setPosition(0.5);
        }
        else{
            hookServo.setPosition(0);
        }
        bPressed = gamepad2.b;

        if (gamepad2.a && !aPressed) {
            aStart = !aStart;
        }
        if (aStart) {
            clawServo.setPosition(0.5);
        }
        else{
            clawServo.setPosition(0);
        }
        aPressed = gamepad2.a;

        if (gamepad1.left_trigger > 0.5) {
            arr = new double[]{-1, 1, -1, 1};
        } else if (gamepad1.right_trigger > 0.5) {
            arr = new double[]{1, -1, 1, -1};
        }

        if(gamepad1.dpad_down){ arr[0] = -1;arr[1] = -1;arr[2] = -1;arr[3] = -1; }
        else if(gamepad1.dpad_right){ arr[0] = 1;arr[1] = -1;arr[2] = -1;arr[3] = 1; }
        else if(gamepad1.dpad_up){ arr[0] = 1;arr[1] = 1;arr[2] = 1;arr[3] = 1; }
        else if(gamepad1.dpad_left){ arr[0] = -1;arr[1] = 1;arr[2] = 1;arr[3] = -1;}

        // SLOW MODE
        if (gamepad1.a && !slowStart) {
            slowStart = !slowStart;
        }
        if (slowStart) {
            arr[0] /= 4.5;
            arr[1] /= 4.5;
            arr[2] /= 4.5;
            arr[3] /= 4.5;
        }
        slowStart = gamepad1.a;

        // Send calculated power to wheels
        bottomleftDrive.setPower(arr[2]);
        bottomrightDrive.setPower(arr[3]);
        topleftDrive.setPower(arr[0]);
        toprightDrive.setPower(arr[1]);
        elevatorMotorleft.setPower(gamepad2.left_stick_y);
        elevatorMotorright.setPower(gamepad2.left_stick_y);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "tleft (%.2f), tright (%.2f), bleft (%.2f), bright (%.2f), ANGLE (%.2f), X (%.2f), Y (%.2f)",
                arr[0], arr[1], arr[2], arr[3], angle, leftStickX, leftStickY);



    }

    public void stop() {
    }
}




