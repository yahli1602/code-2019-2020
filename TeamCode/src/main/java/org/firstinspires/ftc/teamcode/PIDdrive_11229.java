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

@Autonomous(name = "PID 11229", group = "PID")

public class PIDdrive_11229 extends LinearOpMode {
    DcMotor lDrive1, lDrive2, rDrive1, rDrive2, slide1, elevator;
    BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    double globalAngle, correction, rotation;
    boolean aButton, bButton, touched;
    PIDcon dRPID = new PIDcon();
    PIDcon dLPID = new PIDcon();
    PIDcon pidRotate = new PIDcon();
    PIDcon aPID = new PIDcon();
    PIDcon sPID = new PIDcon();
    PIDcon ScPID = new PIDcon();

    PIDcon a_pidDrive = new PIDcon();
    double d_error = 0;
    double d_prevError = 0;
    double d_RstartPoint = 0;
    double d_LstartPoint = 0;
    double a_startPoint = 0;
    double s_startPoint = 0;
    double sc_startPoint = 0;
    double coraction;
    double Scoraction;
    double RcuurentPosition = 0;
    double LcuurentPosition = 0;
    double ScurrentPosition = 0;
    double d_Rpower = 0;
    double d_Lpower = 0;

    int h = 0;
    int f = 0;
    double integral = 0;
    double derivative = 0;

    private final double perimeter = 4 * Math.PI;
    private final double ticksPerRevolution = 28;
    private final double inchesPerTick = perimeter / ticksPerRevolution;
    private final double ticksPerSpin = ticksPerRevolution * 20;
    private final double ticksPerInch = 1 / perimeter * ticksPerSpin;
    private final double SticksPerSpin = ticksPerRevolution * 40;
    private final double SticksPerInch = 1 / perimeter * ticksPerSpin;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
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

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C 0port
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


        dRPID.PIDcon(0.1, 0, 0.45);
        dLPID.PIDcon(0.1, 0, 0.45);

        sPID.PIDcon(0.01, 0, 0);

        aPID.PIDcon(0.08, 0, 0);


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

        f = 0;
        while (opModeIsActive() && f == 0) {
            driveInches(90, -0.5, 0.5);
            f++;

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


    private void driveInches(double inches, double minimumP, double maximumP) {

        double d_Rpower = 0;
        double d_Lpower = 0;

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


        dRPID.setSetPoint(inches);
        dRPID.setOutputRange(minimumP, maximumP);

        dLPID.setSetPoint(inches);
        dLPID.setOutputRange(minimumP, maximumP);

        aPID.setSetPoint(0);
        aPID.setOutputRange(-0.04, 0.04);


        d_RstartPoint = rDrive1.getCurrentPosition();

        d_LstartPoint = lDrive1.getCurrentPosition();

        a_startPoint = getAngle();


        RcuurentPosition = rDrive1.getCurrentPosition() / ticksPerInch;
        dRPID.setSensorValue(RcuurentPosition);
        dRPID.calculate();

        LcuurentPosition = lDrive1.getCurrentPosition() / ticksPerInch;
        dLPID.setSensorValue(LcuurentPosition);
        dLPID.calculate();


        while (dRPID.getError() > 0 && dLPID.getError() > 0 && opModeIsActive()) {


            RcuurentPosition = (rDrive1.getCurrentPosition() / ticksPerInch);
            dRPID.setSensorValue(RcuurentPosition);
            d_Rpower = dRPID.calculate();

            LcuurentPosition = (lDrive1.getCurrentPosition() / ticksPerInch);
            dRPID.setSensorValue(LcuurentPosition);
            d_Lpower = dLPID.calculate();

            aPID.setSensorValue(getAngle());
            coraction = aPID.calculate();


            telemetry.update();

            // set power levels.`
            lDrive1.setPower(d_Lpower);// - coraction);
            lDrive2.setPower(d_Lpower);// - coraction);

            rDrive1.setPower(d_Rpower);// + coraction);
            rDrive2.setPower(d_Rpower);// + coraction);


            telemetry.addData("left power", lDrive2.getPower());
            telemetry.addData("right power", rDrive2.getPower());
            telemetry.addData("RcurrentP", rDrive1.getCurrentPosition());
            telemetry.addData("LcurrentP", lDrive1.getCurrentPosition());
            telemetry.update();

            sleep(15);


        }


        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        telemetry.addData("motors", "off");


    }


    private void slideInches(double inches, double minimumPs, double maximumPs) {

        double d_Spower = 0;


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


        sPID.setSetPoint(inches);
        sPID.setOutputRange(minimumPs, maximumPs);


        ScPID.setSetPoint(0);
        ScPID.setOutputRange(-0.04, 0.04);


        s_startPoint = slide1.getCurrentPosition();

        sc_startPoint = getAngle();


        ScurrentPosition = slide1.getCurrentPosition() / SticksPerInch;
        sPID.setSensorValue(ScurrentPosition);
        sPID.calculate();


        while (sPID.getError() > 0 && opModeIsActive()) {


            ScurrentPosition = (slide1.getCurrentPosition() - s_startPoint) / SticksPerInch;
            sPID.setSensorValue(ScurrentPosition);
            d_Spower = sPID.calculate();


            telemetry.update();

            // set power levels.`
            slide1.setPower(d_Spower);


            if (d_Spower < 0.1 && opModeIsActive()) {
                sPID.setSensorValue(inches);
                sPID.calculate();
            }

            telemetry.addData("slide error", sPID.getError());


        }


        slide1.setPower(0);
        telemetry.addData("motors", "off");


    }


}