// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Drive Avoid PID", group="Exercises")
//@Disabled
public class PIDAutonTest extends LinearOpMode
{
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, rotation;
    boolean                 aButton, bButton, touched;
    PIDController           pidRotate, pidDrive, pidDistance;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.0045, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(0.005, 0, 0);

        pidDistance = new PIDController(0.005, 0.00002, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // Set up parameters for driving in a straight line.
//        pidDrive.setSetpoint(0);
//        pidDrive.setOutputRange(0, power);
//        pidDrive.setInputRange(-90, 90);
//        pidDrive.enable();


//        correction = pidDrive.performPID(getAngle());
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.addData("4 turn rotation", rotation);
        telemetry.update();

//        leftFront.setPower(power - correction);
//        leftRear.setPower(power - correction);
//        rightFront.setPower(power + correction);
//        rightRear.setPower(power + correction);
        // drive until end of period.

//        drive(24, 0.3);
        rotate(-90, power);

//        while (opModeIsActive())
//        {
//            // Use PID with imu input to drive in a straight line.
//            correction = pidDrive.performPID(getAngle());
//
//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
//            telemetry.addData("2 global heading", globalAngle);
//            telemetry.addData("3 correction", correction);
//            telemetry.addData("4 turn rotation", rotation);
//            telemetry.update();
//
//            // set power levels.
//            leftFront.setPower(power - correction);
//            leftRear.setPower(power - correction);
//            rightFront.setPower(power + correction);
//            rightRear.setPower(power + correction);
//
//            // We record the sensor values because we will test them in more than
//            // one place with time passing between those places. See the lesson on
//            // Timing Considerations to know why.
//
//            aButton = gamepad1.a;
//            bButton = gamepad1.b;
//
//            if (touched || aButton || bButton)
//            {
////                // backup.
////                leftMotor.setPower(-power);
////                rightMotor.setPower(-power);
////
////                sleep(500);
////
////                // stop.
////                leftMotor.setPower(0);
////                rightMotor.setPower(0);
//
//                // turn 90 degrees right.
//                if (touched || aButton) ;
//
//                // turn 90 degrees left.
//                if (bButton) rotate(90, power);
//            }
//        }

        // turn the motors off.
//        rightFront.setPower(0);
//        rightRear.setPower(0);
//        leftFront.setPower(0);
//        leftRear.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void drive(double inches, double power) {
//        resetAngle();
//        pidDrive.reset();
//        pidDrive.setSetpoint(0);
//        pidDrive.setOutputRange(0, power);
//        pidDrive.setInputRange(-90, 90);
//        pidDrive.enable();      (int) ((1440/(3.14159265*4)) * inches

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int motorCounts = Math.abs(rightFront.getCurrentPosition()) + 1440;
        pidDistance.reset();
        pidDistance.setSetpoint(motorCounts);
        pidDistance.setInputRange(Math.abs(rightFront.getCurrentPosition()), motorCounts);
        pidDistance.setOutputRange(0, power);

//        pidDistance.setTolerance(0);
        pidDistance.enable();

//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        leftFront.setTargetPosition(-(int)inches);
//
//        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // 4/1440 = inches/x




        do
        {
//            correction = pidDrive.performPID(getAngle());
            double motorPower = pidDistance.performPID(Math.abs(rightFront.getCurrentPosition()));
//            leftFront.setPower(motorPower - correction);
//            leftRear.setPower(motorPower - correction);
//            rightFront.setPower(motorPower + correction);
//            rightRear.setPower(motorPower + correction);
            leftFront.setPower(motorPower);
            leftRear.setPower(motorPower);
            rightFront.setPower(motorPower);
            rightRear.setPower(motorPower);

            telemetry.addData("position: ", Math.abs(rightFront.getCurrentPosition()));
            telemetry.addData("power: ", motorPower);
            telemetry.update();
        } while (opModeIsActive() && !pidDistance.onTarget());
//        while (opModeIsActive() && leftFront.isBusy()) {
//            idle();
//        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output 
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                leftFront.setPower(power);
                leftRear.setPower(power);
                rightFront.setPower(-power);
                rightRear.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                leftFront.setPower(-power);
                leftRear.setPower(-power);
                rightFront.setPower(power);
                rightRear.setPower(power);
                telemetry.addData("power", power);
                telemetry.update();
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                leftFront.setPower(-power);
                leftRear.setPower(-power);
                rightFront.setPower(power);
                rightRear.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}