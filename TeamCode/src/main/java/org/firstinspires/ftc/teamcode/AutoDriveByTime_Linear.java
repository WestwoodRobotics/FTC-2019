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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous(name="Auto Drive By Time")
public class AutoDriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor bottomleftDrive = null;
    private DcMotor bottomrightDrive = null;
    private DcMotor topleftDrive = null;
    private DcMotor toprightDrive = null;

    private DcMotor elevatorMotor = null;

    private Servo flipperServo1 = null;
    private Servo flipperServo2 = null;

    private DcMotor vacuumMotor = null;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        bottomleftDrive = hardwareMap.get(DcMotor.class, "bottom_left_drive");
        bottomrightDrive = hardwareMap.get(DcMotor.class, "bottom_right_drive");
        topleftDrive = hardwareMap.get(DcMotor.class, "top_left_drive");
        toprightDrive = hardwareMap.get(DcMotor.class, "top_right_drive");

        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");

        flipperServo1 = hardwareMap.get(Servo.class, "flipper_servo1");
        flipperServo2 = hardwareMap.get(Servo.class, "flipper_servo2");

        vacuumMotor = hardwareMap.get(DcMotor.class, "vacuum_motor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        bottomleftDrive.setDirection(DcMotor.Direction.FORWARD);
        bottomrightDrive.setDirection(DcMotor.Direction.REVERSE);
        topleftDrive.setDirection(DcMotor.Direction.FORWARD);
        toprightDrive.setDirection(DcMotor.Direction.REVERSE);

        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);

        flipperServo1.setDirection(Servo.Direction.FORWARD);
        flipperServo2.setDirection(Servo.Direction.REVERSE);

        vacuumMotor.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        bottomleftDrive.setPower(FORWARD_SPEED);
        bottomrightDrive.setPower(FORWARD_SPEED);
        topleftDrive.setPower(FORWARD_SPEED);
        toprightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        bottomleftDrive.setPower(-TURN_SPEED);
        bottomrightDrive.setPower(TURN_SPEED);
        topleftDrive.setPower(-TURN_SPEED);
        toprightDrive.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        bottomleftDrive.setPower(FORWARD_SPEED);
        bottomrightDrive.setPower(FORWARD_SPEED);
        topleftDrive.setPower(FORWARD_SPEED);
        toprightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();

            // Step 4:  Stop and close the claw.
            bottomleftDrive.setPower(0);
            bottomrightDrive.setPower(0);
            topleftDrive.setPower(0);
            toprightDrive.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }
}
