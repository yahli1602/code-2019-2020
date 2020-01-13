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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "PID drive 11226 2", group = "PID")

public class
PIDdrive_11226_2 extends LinearOpMode {
    DcMotor ldrive1, ldrive2, rdrive1, rdrive2, slide1, slide2, elevator;
    CRServo hold;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .07, correction, rotation;
    boolean aButton, bButton, touched;
    PIDcon dPID = new PIDcon();
    PIDcon pidRotate = new PIDcon();
    double d_error = 0;
    private double d_maximumOutput = 0.7;    // |maximum output|
    private double d_minimumOutput = -0.7;    // |minimum output|
    int sign;
    double d_prevError = 0;
    double d_startPoint = 0;
    double d_setpoint = 0;
    double cuurentPosition = 0;
    double integral = 0;
    double derivative = 0;
    double r_dPower = 0;
    double l_dPower = 0;


    double d_kP = 0.1;
    double d_kI = 0;
    double d_kD = 0;

    /*private final double perimeter = 4 * Math.PI;
    private final double ticksPerRevolution = 1120;
    private final double inchesPerTick = perimeter / ticksPerRevolution;
    private final double ticksPerSpin = ticksPerRevolution * 40;
    private final double ticksPerInch = 1 / inchesPerTick;*/

    private final double perimeter = 4 * Math.PI;
    private final double ticksPerRevolution = 28;
    private final double inchesPerTick = perimeter / ticksPerRevolution;
    private final double ticksPerSpin = ticksPerRevolution * 25;
    private final double ticksPerInch = 1 / perimeter * ticksPerSpin;


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        rdrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        ldrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");
        hold = hardwareMap.get(CRServo.class, "hold");

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        /*collectRight = hardwareMap.get(Servo.class, "collect right");
        collectLeft = hardwareMap.get(Servo.class, "collect left");
        cubeIn = hardwareMap.get(TouchSensor.class, "cube in");*/

        rdrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rdrive1.setDirection(DcMotor.Direction.FORWARD);
        rdrive2.setDirection(DcMotor.Direction.FORWARD);
        ldrive1.setDirection(DcMotor.Direction.REVERSE);
        ldrive2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);

        rdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rdrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rdrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to REV Touch sensor.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate.PIDcon(0, 0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.


        dPID.PIDcon(0.2, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
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
        int q = 0;
        while (opModeIsActive() && q == 0) {

            driveInches(48, 0.7);
            q++;

        }
        // Use PID with imu input to drive in a straight line.

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
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
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
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

        pidRotate.setOutputRange(0, power);


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                ldrive1.setPower(power);
                ldrive2.setPower(power);
                rdrive1.setPower(-power);
                rdrive2.setPower(-power);
                sleep(100);
            }

            do {
                power = pidRotate.calculate(); // power will be - on right turn.
                ldrive1.setPower(-power);
                ldrive2.setPower(-power);
                rdrive1.setPower(power);
                rdrive2.setPower(power);

            } while (opModeIsActive());
        } else    // left turn.
            do {
                power = pidRotate.calculate(); // power will be + on left turn.
                ldrive1.setPower(-power);
                ldrive2.setPower(-power);
                rdrive1.setPower(power);
                rdrive2.setPower(power);
            } while (opModeIsActive());

        // turn the motors off.
        rdrive1.setPower(0);
        rdrive2.setPower(0);
        ldrive1.setPower(0);
        ldrive2.setPower(0);


        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }


    private void driveInches(double inches, double d_power) {


        dPID.setOutputRange(-d_power, d_power);

        dPID.setSetPoint(inches);


        d_startPoint = rdrive1.getCurrentPosition();


        cuurentPosition = (rdrive1.getCurrentPosition() - d_startPoint) / ticksPerInch;
        dPID.setSensorValue(cuurentPosition);

        dPID.calculate();

        while (d_error != 0 && opModeIsActive()) {

            cuurentPosition = (rdrive1.getCurrentPosition() - d_startPoint) / ticksPerInch;
            dPID.setSensorValue(cuurentPosition);

            d_power = dPID.calculate();


            telemetry.addData("dError", d_error);
            telemetry.addData("cPosition", cuurentPosition);
            telemetry.addData("dPower", d_power);


            telemetry.update();

            // set power levels.


            if (d_power > 0) sign = 1;
            else if (d_power < 0) sign = -1;


            if (sign == 1) {
                if (getAngle() > 0) {
                    r_dPower = 0.17;
                    l_dPower = 0;

                } else if (getAngle() < 0) {
                    l_dPower = 0.17;
                    r_dPower = 0;
                }
            } else if (sign == -1) {
                if (getAngle() > 0) {
                    r_dPower = 0;
                    l_dPower = -0.17;

                } else if (getAngle() < 0) {
                    l_dPower = 0;
                    r_dPower = -0.17;
                }
            }

            if (getAngle() == 0) {
                l_dPower = 0;
                r_dPower = 0;
            }


            ldrive1.setPower(d_power + l_dPower);
            ldrive2.setPower(d_power + l_dPower);
            rdrive1.setPower(d_power + r_dPower);
            rdrive2.setPower(d_power + r_dPower);

            telemetry.addData("dError", d_error);
            telemetry.addData("cPosition", cuurentPosition);
            telemetry.addData("dPower", d_power);

            sleep(15);

            telemetry.addData("left position", ldrive2.getCurrentPosition());
            telemetry.addData("right position", rdrive2.getCurrentPosition());
            telemetry.addData("left power", ldrive2.getPower());
            telemetry.addData("right power", rdrive2.getPower());
        }


        ldrive1.setPower(0);
        ldrive2.setPower(0);
        rdrive1.setPower(0);
        rdrive2.setPower(0);


    }


}