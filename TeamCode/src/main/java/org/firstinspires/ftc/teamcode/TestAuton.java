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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 * This is our auton.
 */

@Autonomous(name="TestAuton", group="Linear Opmode")
public class TestAuton extends LinearOpMode {

    private static final String VUFORIA_KEY =
            "AVh3cr//////AAABmWeVEMeTuUwztvRE/TbUsG1tdSWUOpFTl75IeigpwiBVVn2HMsE9qGRdLaR04XBlxiDHbect9LJPJBwrRz17gmGrIXnVJ0ob/m3TDL4IWv8/gUO/HTmDSWaJwE4Q4ZL9ek1uFJKBrxHnoQUQicoyaGME1RyIC7BkQguGNK+8b+NUWWo0o4u3kdNcRq8WqUcmBQqydjEeZniTJm4PMRf4ReFt5Rb58y39gkv1CxxociIbwXPg7gnRJNSiaQBg2jcnk/NGTo/TFv6cuHQnwhvj92GQWNjgry1AebIa1vD2+4Am62LjAyLy+NW71kMbPGo/sXhjQ311fchEnmPrZTlbUk3cIxYpv79F58Vu2cIxcizX";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private static final float stoneZ = 2.00f * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    private final double COUNTS_PER_INCH = 37.2093023256;
    private final double COUNTS_PER_INCH_STRAFE = 44.4290177022;


    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private Servo armServo;
    private Servo grabServo;
    private int statusNum = 0;

    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled      = true;
        imuParameters.loggingTag          = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        armServo = hardwareMap.get(Servo.class, "servo");
        grabServo = hardwareMap.get(Servo.class, "grabServo");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        float offsetAngle = 0;
        int block = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
//            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            switch (statusNum) {

                case 0:
                    encoderDrive(0.4, 22, "r");
                case 1:
                    encoderDrive(0.2, 3 , "b");
                case 2:
                    targetsSkyStone.activate();

                    runtime.reset();

                    while (runtime.time() < 1)

                    targetVisible = false;
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }

                    // Provide feedback as to where the robot is located (if we know).
                    if (targetVisible) {
                        // express position (translation) of robot in inches.
                        VectorF translation = lastLocation.getTranslation();
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                        telemetry.update();
                        block = 16;
                        encoderDrive(0.2,1,"b");
                        encoderDrive(0.2, 1, "r");
                    }
                    else {
                        telemetry.addData("Visible Target", "none");
                        encoderDrive(0.2, 7, "f");
                        runtime.reset();

                        while (runtime.time() < 1)

                        for (VuforiaTrackable trackable : allTrackables) {
                            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                                telemetry.addData("Visible Target", trackable.getName());
                                targetVisible = true;

                                // getUpdatedRobotLocation() will return null if no new information is available since
                                // the last time that call was made, or if the trackable is not currently visible.
                                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                                if (robotLocationTransform != null) {
                                    lastLocation = robotLocationTransform;
                                }
                                break;
                            }
                        }

                        if (targetVisible) {
                            // express position (translation) of robot in inches.
                            VectorF translation = lastLocation.getTranslation();
                            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                            // express the rotation of the robot in degrees.
                            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                            telemetry.update();
                            block = 8;
                            encoderDrive(0.2, 1, "r");
                        }
                        else {
                            telemetry.addData("Visible Target", "none");
                            encoderDrive(0.2, 6, "f");
                            encoderDrive(0.2, 1, "r");


                        }
                    }


                    statusNum++;

                case 3:

                    armServo.setPosition(0);
                    grabServo.setPosition(1.0);
                    sleep(2150);


                    grabServo.setPosition(0.5);
                    armServo.setPosition(0.5);
                    statusNum++;
                case 4:
                    encoderDrive(0.2, 2, "r");
                    grabServo.setPosition(0);
                    sleep(2300);
                    grabServo.setPosition(0.3);
                    armServo.setPosition(1.0);
                    sleep(2200);
                    armServo.setPosition(0.5);
                    statusNum++;
                case 5:
                    encoderDrive(0.3, 3, "l");
                    encoderDrive(0.5, 68 + block, "f");
                    encoderDrive(0.3, 6, "r");


                    armServo.setPosition(0);
                    sleep(2200);
                    armServo.setPosition(0.5);
                    grabServo.setPosition(1.0);
                    sleep(1000);
                    grabServo.setPosition(0.5);
                    armServo.setPosition(1.0);
                    sleep(2200);
                    armServo.setPosition(0.5);
                    encoderDrive(0.3, 9, "l");
                    encoderDrive(0.5, 31, "b");
                    statusNum++;
                default:
                    break;
            }

//            telemetry.addData("Servo", "value (%f)", angles.firstAngle);
//            telemetry.update();
        }

//        while (angles.firstAngle < 86) {
//            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//        }
//        leftDrive.setPower(0);
//        rightDrive.setPower(0);
//        retractFromLander();
//        encoderDrive(DRIVE_SPEED,  4,  4);

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void encoderDrive(double speed, double inches, String direction) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            int leftFrontTarget = leftFront.getCurrentPosition();
            int leftRearTarget = leftRear.getCurrentPosition();
            int rightFrontTarget = rightFront.getCurrentPosition();
            int rightRearTarget = rightRear.getCurrentPosition();

            switch(direction) {
                case "f":
                    leftFrontTarget += inches * COUNTS_PER_INCH;
                    leftRearTarget += inches * COUNTS_PER_INCH;
                    rightFrontTarget += inches * COUNTS_PER_INCH;
                    rightRearTarget += inches * COUNTS_PER_INCH;

                    while (opModeIsActive() && (leftFront.getCurrentPosition() < leftFrontTarget && leftRear.getCurrentPosition() < leftRearTarget && rightFront.getCurrentPosition() < rightFrontTarget && rightRear.getCurrentPosition() < rightRearTarget)) {

                        leftFront.setPower(-speed);
                        leftRear.setPower(-speed);
                        rightFront.setPower(-speed);
                        rightRear.setPower(-speed);

                        telemetry.addData("Encoder", leftFront.getCurrentPosition());
                        telemetry.addData("Encoder", leftRear.getCurrentPosition());
                        telemetry.addData("Encoder", rightFront.getCurrentPosition());
                        telemetry.addData("Encoder", rightRear.getCurrentPosition());
                        telemetry.update();
                    }
                    break;
                case "b":
                    leftFrontTarget -= inches * COUNTS_PER_INCH;
                    leftRearTarget -= inches * COUNTS_PER_INCH;
                    rightFrontTarget -= inches * COUNTS_PER_INCH;
                    rightRearTarget -= inches * COUNTS_PER_INCH;

                    leftFront.setPower(speed);
                    leftRear.setPower(speed);
                    rightFront.setPower(speed);
                    rightRear.setPower(speed);

                    while (opModeIsActive() && (leftFront.getCurrentPosition() > leftFrontTarget && leftRear.getCurrentPosition() > leftRearTarget && rightFront.getCurrentPosition() > rightFrontTarget && rightRear.getCurrentPosition() > rightRearTarget)) {
                        telemetry.addData("Encoder", leftFront.getCurrentPosition());
                        telemetry.addData("Encoder", leftRear.getCurrentPosition());
                        telemetry.addData("Encoder", rightFront.getCurrentPosition());
                        telemetry.addData("Encoder", rightRear.getCurrentPosition());
                        telemetry.update();
                    }
                    break;
                case "r":
                    leftFrontTarget += inches * COUNTS_PER_INCH_STRAFE;
                    leftRearTarget -= inches * COUNTS_PER_INCH_STRAFE;
                    rightFrontTarget -= inches * COUNTS_PER_INCH_STRAFE;
                    rightRearTarget += inches * COUNTS_PER_INCH_STRAFE;

                    leftFront.setPower(-speed);
                    leftRear.setPower(speed);
                    rightFront.setPower(speed);
                    rightRear.setPower(-speed);

                    while (opModeIsActive() && (leftFront.getCurrentPosition() < leftFrontTarget && leftRear.getCurrentPosition() > leftRearTarget && rightFront.getCurrentPosition() > rightFrontTarget && rightRear.getCurrentPosition() < rightRearTarget)) {
                        telemetry.addData("Encoder", leftFront.getCurrentPosition());
                        telemetry.addData("Encoder", leftRear.getCurrentPosition());
                        telemetry.addData("Encoder", rightFront.getCurrentPosition());
                        telemetry.addData("Encoder", rightRear.getCurrentPosition());
                        telemetry.update();
                    }
                    break;
                case "l":
                    leftFrontTarget -= inches * COUNTS_PER_INCH_STRAFE;
                    leftRearTarget += inches * COUNTS_PER_INCH_STRAFE;
                    rightFrontTarget += inches * COUNTS_PER_INCH_STRAFE;
                    rightRearTarget -= inches * COUNTS_PER_INCH_STRAFE;

                    leftFront.setPower(speed);
                    leftRear.setPower(-speed);
                    rightFront.setPower(-speed);
                    rightRear.setPower(speed);

                    while (opModeIsActive() && (leftFront.getCurrentPosition() > leftFrontTarget && leftRear.getCurrentPosition() < leftRearTarget && rightFront.getCurrentPosition() < rightFrontTarget && rightRear.getCurrentPosition() > rightRearTarget)) {
                        telemetry.addData("Encoder", leftFront.getCurrentPosition());
                        telemetry.addData("Encoder", leftRear.getCurrentPosition());
                        telemetry.addData("Encoder", rightFront.getCurrentPosition());
                        telemetry.addData("Encoder", rightRear.getCurrentPosition());
                        telemetry.update();
                    }
                    break;
                default:
                    break;
            }

            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            sleep(100);   // pause after each move
            statusNum++;
        }
    }
}
