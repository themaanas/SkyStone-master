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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Linear Opmode")
public class teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftRear, rightRear, liftMotor, rightIntake, leftIntake;
    private Servo armServo, rightArmServo, liftServo;
    private TouchSensor limitSwitch;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        rightIntake = hardwareMap.get(DcMotor.class, "rightCore");
        leftIntake = hardwareMap.get(DcMotor.class, "leftCore");
        armServo = hardwareMap.get(Servo.class, "arm");
        rightArmServo = hardwareMap.get(Servo.class, "right");
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        limitSwitch = hardwareMap.get(TouchSensor.class, "switch");

        // set the digital channel to input.
//        armServo = hardwareMap.get(Servo.class, "servo");
//        grabServo = hardwareMap.get(Servo.class,"grabServo");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        boolean goDown = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//
//            if (gamepad1.y)
//                armServo.setPosition(1.0);
//            else if (gamepad1.a)
//                armServo.setPosition(0);
//            else
//                //The value 0.523 is the value to make the servo hold its position.
//                armServo.setPosition(0.523);
//
//            if (gamepad1.x)
//                grabServo.setPosition(1.0);
//            else if (gamepad1.b)
//                grabServo.setPosition(0);
//            else
//                //Same with above, this makes it hold its position.
//                grabServo.setPosition(0.50);

            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double rightX = gamepad1.right_stick_x;
            double v1 = -r * Math.sin(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI / 4) + rightX;
            double v2 = r * Math.sin(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4) + rightX;
            double v3 = -r * Math.sin(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4) + rightX;
            double v4 = r * Math.sin(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI / 4) + rightX;


            if (gamepad1.right_bumper) {
                v1 /= 2;
                v2 /= 2;
                v3 /= 2;
                v4 /= 2;
            }
            if (gamepad1.x) {
                armServo.setPosition(1.0);
                rightArmServo.setPosition(0);
                sleep(1000);
            } else if (gamepad1.b) {
                armServo.setPosition(0);
                rightArmServo.setPosition(1.0);
                sleep(1000);
            }

            double liftPower = 0;
            if (gamepad1.y) {
                liftPower = 0.7;
            } else if (gamepad1.a && !limitSwitch.isPressed()) {
                liftPower = -0.7;
            }


            if (gamepad1.dpad_down && !limitSwitch.isPressed()) {
                goDown = true;
            }
            if (goDown) {
                liftPower = -0.7;
            }

            double intakePower = 0;
            if (gamepad1.right_trigger > 0) {
                intakePower = -1;
            } else if (gamepad1.left_trigger > 0) {
                intakePower = 1;
            }


            // 0 is open, 1 is close, 0.72 is almost close

            if (gamepad1.dpad_left) {
                liftServo.setPosition(0.4);
                sleep(500);
            } else if (gamepad1.dpad_up) {
                liftServo.setPosition(0.65);
                sleep(500);
            } else if (gamepad1.dpad_right) {

                liftServo.setPosition(1.0);
                sleep(500);
            }
            if (limitSwitch.isPressed()) {

                goDown = false;
            }
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
            liftMotor.setPower(liftPower);
            rightIntake.setPower(intakePower);
            leftIntake.setPower(-intakePower);
            telemetry.addData("Digital Touch", limitSwitch.isPressed());
            telemetry.addData("Digital Name", limitSwitch.getDeviceName());


            telemetry.addData("coords", "x (%.2f), y (%.2f)", gamepad1.left_stick_x, -gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
