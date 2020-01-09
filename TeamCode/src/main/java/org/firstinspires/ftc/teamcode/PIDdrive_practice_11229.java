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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="PID 11229", group="PID")

public class PIDdrive_practice_11229 extends LinearOpMode
{
    DcMotor                 lDrive1,lDrive2,rDrive1,rDrive2,slide1,elevator;
    BNO055IMU               imu;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .07, correction, rotation;
    boolean                 aButton, bButton, touched;
    PIDController           pidRotate, a_pidDrive,dPID;
    double                  d_error = 0;
    double                  d_prevError = 0;
    double                  d_startPoint = 0;
    double cuurentPosition = 0;
    int h = 0;
    int f = 0;
    double integral = 0;
    double derivative = 0;

    private final double perimeter = 4 * Math.PI;
    private final double ticksPerRevolution = 1120;
    private final double inchesPerTick = perimeter / ticksPerRevolution;
    private final double ticksPerSpin = ticksPerRevolution * 20;
    private final double ticksPerInch = 1 / inchesPerTick;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide1 = hardwareMap.get(DcMotor.class, "slide");

        elevator = hardwareMap.get(DcMotor.class, "elevator");



        rDrive1.setDirection(DcMotor.Direction.FORWARD);
        rDrive2.setDirection(DcMotor.Direction.FORWARD);
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);


        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to REV Touch sensor.

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
        pidRotate = new PIDController(0,0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        a_pidDrive = new PIDController(0, 0, 0);

        dPID = new PIDController(0.15,0,0);
        dPID.enable();

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





        // drive until end of period.

        f = 0;
        while (opModeIsActive() && f == 0)
        {
            driveInches(44,0.7);
            f++;

        }
            // Use PID with imu input to drive in a straight line.

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
                lDrive1.setPower(power);
                lDrive2.setPower(power);
                rDrive1.setPower(-power);
                rDrive2.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                lDrive1.setPower(-power);
                lDrive2.setPower(-power);
                rDrive1.setPower(power);
                rDrive2.setPower(power);

            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                lDrive1.setPower(-power);
                lDrive2.setPower(-power);
                rDrive1.setPower(power);
                rDrive2.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);


        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }



    private void driveInches(double inches, double d_power) {

        h = 0;



        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        dPID.setSetpoint(inches);
        dPID.setOutputRange(-0.7, 0.7);





        d_startPoint = rDrive1.getCurrentPosition();




        cuurentPosition = rDrive1.getCurrentPosition()/ ticksPerInch;
        dPID.setInput(cuurentPosition);
        dPID.performPID();


        while (dPID.getError() > 0 && opModeIsActive()) {

            cuurentPosition = (rDrive1.getCurrentPosition() - d_startPoint)/ ticksPerInch;
            dPID.setInput(cuurentPosition);
            d_power = dPID.performPID();


            telemetry.addData("d_error",dPID.getError());
            telemetry.addData("cPosition",cuurentPosition);
            telemetry.addData("error",dPID.getError());
            telemetry.addData("dPower",d_power);


            telemetry.update();

            // set power levels.`
            lDrive1.setPower(d_power);
            lDrive2.setPower(d_power);

            if (h == 0 && opModeIsActive()){
                sleep(75);
                h++;
            }

            rDrive1.setPower(d_power);
            rDrive2.setPower(d_power);

            telemetry.addData("left position", lDrive2.getCurrentPosition());
            telemetry.addData("right position", rDrive2.getCurrentPosition());
            telemetry.addData("left power", lDrive2.getPower());
            telemetry.addData("right power", rDrive2.getPower());
        }


        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);


    }



}