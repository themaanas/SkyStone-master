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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="Drive Avoid PID", group="Exercises")
//@Disabled
public class PIDAutonTest extends LinearOpMode
{

    private static final String VUFORIA_KEY =
            "AVh3cr//////AAABmWeVEMeTuUwztvRE/TbUsG1tdSWUOpFTl75IeigpwiBVVn2HMsE9qGRdLaR04XBlxiDHbect9LJPJBwrRz17gmGrIXnVJ0ob/m3TDL4IWv8/gUO/HTmDSWaJwE4Q4ZL9ek1uFJKBrxHnoQUQicoyaGME1RyIC7BkQguGNK+8b+NUWWo0o4u3kdNcRq8WqUcmBQqydjEeZniTJm4PMRf4ReFt5Rb58y39gkv1CxxociIbwXPg7gnRJNSiaQBg2jcnk/NGTo/TFv6cuHQnwhvj92GQWNjgry1AebIa1vD2+4Am62LjAyLy+NW71kMbPGo/sXhjQ311fchEnmPrZTlbUk3cIxYpv79F58Vu2cIxcizX";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private static final float stoneZ = 2.00f * mmPerInch;

    private ElapsedTime runtime = new ElapsedTime();

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    private VuforiaTrackables targetsSkyStone;
    private List<VuforiaTrackable> allTrackables;

    static final double     COUNTS_PER_INCH = 42.836315087;    // eg: TETRIX Motor Encoder
    static final double     STRAFE_COUNTS_PER_INCH = 42.836315087;    // eg: TETRIX Motor Encoder

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private Servo armServo;
    private Servo rightArmServo;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction, rotation;
    PIDController           pidRotate, pidDrive, pidForward, pidStrafe;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        pidRotate = new PIDController(0.008, 0.00008, 0);
        pidDrive = new PIDController(0.05, 0, 0);
        pidForward = new PIDController(0.00055, 0.000008, 0);
        pidStrafe = new PIDController(0.0008, 0.000008, 0);

        initRobot();
//        initCamera();
        waitForStart();
//        drive(-44, 0.6);




        drive(-9, 0.5);
//        strafe(5, 0.5);
//        drive(18, 0.5);

        String stonePos = "middle";
        switch (stonePos) {
            case "left":

                break;
            case "middle":


                strafe(2, 0.5);
                drive(-13, 0.4);
                rightArmServo.setPosition(1.0);
                sleep(2000);
                drive(10, 0.5);
                rotate(90, 0.8);
                sleep(3000);
                drive(50, 0.5);
                rightArmServo.setPosition(0.5);
                sleep(2000);
//                drive(50, 0.6);
                break;


            case "right":
                strafe(8, 0.4);
                break;
        }
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

    //
    // STRAFE LEFT AND RIGHT METHOD
    //

    private void strafe(double inches, double power) {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int OFFSET_VALUE = 20;
        double pStartValue = 0.008;
        int forward = 1;

        if (inches < 0)
            forward = -1;

        int motorCounts = (int) (forward * inches * COUNTS_PER_INCH);
        int startPos = Math.abs(rightFront.getCurrentPosition()) - OFFSET_VALUE;
        int targetPos = Math.abs(rightFront.getCurrentPosition()) + (motorCounts / 4);

        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 0.2);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();


//        while (opModeIsActive() && Math.abs(rightFront.getCurrentPosition()) < targetPos) {
//            double correction = pidDrive.performPID(getAngle());
//            double motorPower = pStartValue * (Math.abs(rightFront.getCurrentPosition()) - startPos);
//
//            if (motorPower > power)
//                motorPower = power;
//
//            leftFront.setPower((forward * motorPower) + correction);
//            leftRear.setPower((forward * -motorPower) + correction);
//            rightFront.setPower((forward * -motorPower) - correction);
//            rightRear.setPower((forward * motorPower) - correction);
//        }


        pidStrafe.reset();
        int currentPos = Math.abs(rightFront.getCurrentPosition());
        pidStrafe.setSetpoint(currentPos + (motorCounts));
        pidStrafe.setInputRange(currentPos, currentPos + (motorCounts));
        pidStrafe.setOutputRange(0.2, power);
        pidStrafe.setTolerance(1);
        pidStrafe.enable();

        do {
            double correction = pidDrive.performPID(getAngle());
            int position = Math.abs(rightFront.getCurrentPosition());
            double motorPower = pidStrafe.performPID(position);


            leftFront.setPower((forward * motorPower) + correction);
            leftRear.setPower((forward * -motorPower) + correction);
            rightFront.setPower((forward * -motorPower) - correction);
            rightRear.setPower((forward * motorPower) - correction);

            telemetry.addData("setpoint", motorCounts);
            telemetry.addData("position: ", position);
            telemetry.addData("power: ", motorPower);
            telemetry.update();
        } while (opModeIsActive() && !pidStrafe.onTarget());

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        pidDrive.disable();
        pidStrafe.disable();


        rotation = getAngle();
        sleep(200);

        resetAngle();
    }

    //
    // DRIVE FORWARD AND BACKWARD METHOD
    //
    private void drive(double inches, double power) {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidForward.reset();

        telemetry.addData("position (0): ", Math.abs(rightFront.getCurrentPosition()));
        telemetry.update();

        sleep(2000);

        int OFFSET_VALUE = 20;
        double pStartValue = 0.006;
        int forward = 1;

        if (inches < 0)
            forward = -1;

        int motorCounts = (int) (forward * inches * COUNTS_PER_INCH);
        int targetPos = Math.abs(rightFront.getCurrentPosition()) + (motorCounts / 4);

        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 0.2);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();


        while (opModeIsActive() && Math.abs(rightFront.getCurrentPosition()) < targetPos) {
            double correction = pidDrive.performPID(getAngle());
            double motorPower = pStartValue * (Math.abs(rightFront.getCurrentPosition()) + OFFSET_VALUE);

            if (motorPower > power)
                motorPower = power;

            leftFront.setPower((forward * motorPower) + correction);
            leftRear.setPower((forward * motorPower) + correction);
            rightFront.setPower((forward * motorPower) - correction);
            rightRear.setPower((forward * motorPower) - correction);
            telemetry.addData("position: ", Math.abs(rightFront.getCurrentPosition()));
            telemetry.update();
        }



        int currentPos = Math.abs(rightFront.getCurrentPosition());
        pidForward.setSetpoint(currentPos + (motorCounts * 3 / 4));
        pidForward.setInputRange(currentPos, currentPos + (motorCounts * 3 / 4));
        pidForward.setOutputRange(0.1, power);
        pidForward.setTolerance(1);
        pidForward.enable();

        do {
            double correction = pidDrive.performPID(getAngle());
            int position = Math.abs(rightFront.getCurrentPosition());
            double motorPower = pidForward.performPID(position);


            leftFront.setPower((forward * motorPower) + correction);
            leftRear.setPower((forward * motorPower) + correction);
            rightFront.setPower((forward * motorPower) - correction);
            rightRear.setPower((forward * motorPower) - correction);

            telemetry.addData("setpoint", (motorCounts * 3 / 4));
            telemetry.addData("position: ", position);
            telemetry.addData("power: ", motorPower);
            telemetry.update();
        } while (opModeIsActive() && !pidForward.onTarget());

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);


        rotation = getAngle();
        sleep(200);

        pidDrive.disable();
        pidForward.disable();



        resetAngle();
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
        pidRotate.setSetpoint(Math.abs(degrees));
        pidRotate.setInputRange(0, Math.abs(degrees));
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
//            // On right turn we have to get off zero first.
//            while (opModeIsActive() && getAngle() == 0)
//            {
//                leftFront.setPower(-power);
//                leftRear.setPower(-power);
//                rightFront.setPower(power);
//                rightRear.setPower(power);
//                sleep(100);
//            }

            do
            {
                power = pidRotate.performPID(Math.abs(getAngle())); // power will be - on right turn.
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
                power = pidRotate.performPID(Math.abs(getAngle())); // power will be + on left turn.
                leftFront.setPower(power);
                leftRear.setPower(power);
                rightFront.setPower(-power);
                rightRear.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);
        pidRotate.disable();
        // reset angle tracking on new heading.
        resetAngle();
    }

    private void initRobot() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        armServo = hardwareMap.get(Servo.class, "arm");
        rightArmServo = hardwareMap.get(Servo.class, "right");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

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
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
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
    }

    private String getSkystonePosition() {
        String stonePosition = "right";

        targetsSkyStone.activate();

        runtime.reset();

        while (opModeIsActive() && runtime.time() < 1)
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
            // -21.4, -3.4, -4.0
            //
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            telemetry.update();
            if (translation.get(1) < 0) {
                stonePosition = "left";
            } else if (translation.get(1) > 0) {
                stonePosition = "middle";
            }
        }

        return stonePosition;
    }
}