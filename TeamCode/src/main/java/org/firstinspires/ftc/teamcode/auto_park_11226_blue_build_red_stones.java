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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto Park 11226 blue Build and red stones", group = "PID")

public class auto_park_11226_blue_build_red_stones extends LinearOpMode {
    DcMotor ldrive1, ldrive2, rdrive1, rdrive2, slide1, elevator;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .07, rotation;
    boolean aButton, bButton, touched;
    PIDcon dRPID = new PIDcon();
    PIDcon dLPID = new PIDcon();
    PIDcon pidRotate = new PIDcon();
    PIDcon aPID = new PIDcon();
    PIDcon SaPID = new PIDcon();
    PIDcon ePID = new PIDcon();
    PIDcon sPID = new PIDcon();
    PIDcon ScPID = new PIDcon();
    double d_error = 0;
    double s_startPoint = 0;
    double sc_startPoint = 0;
    double Scoraction;
    double ScurrentPosition = 0;
    double RcuurentPosition = 0;
    double LcuurentPosition = 0;
    double d_prevError = 0;
    double d_RstartPoint = 0;
    double d_LstartPoint = 0;


    double integral = 0;
    double derivative = 0;
    double d_Rpower = 0;
    double d_Lpower = 0;
    double d_Spower = 0;
    double coraction = 0;
    double SAcorraction = 0;


    double d_kP = 0.1;
    double d_kI = 0;
    double d_kD = 0;

    boolean Con = true;

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

    private final double SticksPerSpin = ticksPerRevolution * 36;
    private final double SticksPerInch = 1 / perimeter * SticksPerSpin;


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        rdrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        ldrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide1 = hardwareMap.get(DcMotor.class, "slide");

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        /*collectRight = hardwareMap.get(Servo.class, "collect right");
        collectLeft = hardwareMap.get(Servo.class, "collect left");
        cubeIn = hardwareMap.get(TouchSensor.class, "cube in");*/

        ePID.PIDcon(0.1, 0, 0);

        rdrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rdrive1.setDirection(DcMotor.Direction.REVERSE);
        rdrive2.setDirection(DcMotor.Direction.REVERSE);
        ldrive1.setDirection(DcMotor.Direction.FORWARD);
        ldrive2.setDirection(DcMotor.Direction.FORWARD);
        slide1.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.REVERSE);

        rdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rdrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rdrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        dRPID.PIDcon(0.07, 0, 0.03);
        dLPID.PIDcon(0.07, 0, 0.2);

        aPID.PIDcon(0.2, 0, 0.7);

        SaPID.PIDcon(0.2, 0, 0.1);

        sPID.PIDcon(0.5, 0.4, 0.15);
        ScPID.PIDcon(0, 0, 0);

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
            sleep(20000);

            slideInches(30, 0.03, 1);
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

    private void driveInches(double inches, double minPower, double maxPower) {

        d_Rpower = 0;
        d_Lpower = 0;

        dRPID.reset();
        dLPID.reset();
        aPID.reset();

        double Raccelerate = 0.05;
        double Laccelerate = 0.05;

        dRPID.setOutputRange(minPower, maxPower);
        dLPID.setOutputRange(minPower, maxPower);

        dRPID.setSetPoint(inches);
        dLPID.setSetPoint(inches);


        d_RstartPoint = rdrive1.getCurrentPosition();
        d_LstartPoint = ldrive1.getCurrentPosition();


        aPID.setSetPoint(0);
        aPID.setOutputRange(-0.01, 0.01);

        SaPID.setSetPoint(0);
        SaPID.setOutputRange(-0.7, 0.7);


        RcuurentPosition = (rdrive1.getCurrentPosition() - d_RstartPoint) / ticksPerInch;
        LcuurentPosition = (ldrive1.getCurrentPosition() - d_LstartPoint) / ticksPerInch;

        dRPID.setSensorValue(RcuurentPosition);
        dLPID.setSensorValue(LcuurentPosition);

        dRPID.calculate();
        dLPID.calculate();


        int n = 1;


        while (dRPID.getError() > 0 && dLPID.getError() > 0 && opModeIsActive()) {


            RcuurentPosition = (rdrive1.getCurrentPosition()) / ticksPerInch;
            dRPID.setSensorValue(RcuurentPosition);

            LcuurentPosition = (ldrive1.getCurrentPosition()) / ticksPerInch;
            dLPID.setSensorValue(LcuurentPosition);

            aPID.setSensorValue(getAngle());


            d_Lpower = dLPID.calculate();
            d_Rpower = dRPID.calculate();
            coraction = aPID.calculate();// + 0.04 * (LcuurentPosition - RcuurentPosition);


            if (d_Rpower == maxPower && Raccelerate < maxPower) {
                d_Rpower = Raccelerate + coraction;
                Raccelerate += 0.05;
            }
            if (d_Lpower == maxPower && Laccelerate < maxPower) {
                d_Lpower = Laccelerate - coraction;
                Laccelerate += 0.05;
            }

            /*if(d_Rpower < 0.0001 && d_error < inches){
                coraction = 0;
            }*/

            telemetry.update();

            // set power levels.


            if (RcuurentPosition <= (inches * 0.05) && LcuurentPosition <= (inches * 0.05)) {
                ldrive1.setPower(d_Lpower);// / n));
                ldrive2.setPower(d_Lpower);// / n));
                rdrive1.setPower(d_Rpower);// / n));
                rdrive2.setPower(d_Rpower);// / n));
                Con = false;
            } else {


            /*if ((d_error / 9 / 16) % 1 == 0) {
                n++;
            }*/
                if (RcuurentPosition >= (inches * 0.9) && LcuurentPosition >= (inches * 0.9)) {
                    ldrive1.setPower(d_Lpower);// / n));
                    ldrive2.setPower(d_Lpower);// / n));
                    rdrive1.setPower(d_Rpower);// / n));
                    rdrive2.setPower(d_Rpower);// / n));
                    Con = false;
                } else {
                    ldrive1.setPower(d_Lpower - coraction);// / n));
                    ldrive2.setPower(d_Lpower - coraction);// / n));
                    rdrive1.setPower(d_Rpower + coraction);// / n));
                    rdrive2.setPower(d_Rpower + coraction);// / n));
                    Con = true;
                }
            }


            //ldrive1.setPower(d_Lpower - coraction);// / n));
            //ldrive2.setPower(d_Lpower - coraction);// / n));
            //rdrive1.setPower(d_Rpower + coraction);// / n));
            //rdrive2.setPower(d_Rpower + coraction);// / n));

            //if ((d_error / 9 / 16) % 1 == 0) {
            //    n++;
            //}
            /*if (d_Rpower <0.06 && opModeIsActive()){
                dRPID.setSensorValue(inches);
                dRPID.calculate();
            }
            if (d_Lpower <0.06 && opModeIsActive()){
                dLPID.setSensorValue(inches);
                dLPID.calculate();
            }*/


            telemetry.addData("left position", ldrive2.getCurrentPosition());
            telemetry.addData("right position", rdrive2.getCurrentPosition());
            telemetry.addData("left power", ldrive2.getPower());
            telemetry.addData("right power", rdrive2.getPower());
            telemetry.addData("coraction", coraction);
            telemetry.addData("d_rpower", d_Rpower);
            telemetry.addData("d_lpower", d_Lpower);
            telemetry.addData("angle", getAngle());
            telemetry.addData("angle", getAngle());
            telemetry.update();
            sleep(15);

        }


        ldrive1.setPower(0);
        ldrive2.setPower(0);
        rdrive1.setPower(0);
        rdrive2.setPower(0);


    }

    private void backInches(double inches, double minPower, double maxPower) {

        d_Rpower = 0;
        d_Lpower = 0;

        double accelerate = -0.05;

        dRPID.setOutputRange(minPower, maxPower);
        dLPID.setOutputRange(minPower, maxPower);

        dRPID.setSetPoint(inches);
        dLPID.setSetPoint(inches);


        d_RstartPoint = rdrive1.getCurrentPosition();
        d_LstartPoint = ldrive1.getCurrentPosition();


        aPID.setSetPoint(0);
        aPID.setOutputRange(-0.07, 0.07);

        RcuurentPosition = (rdrive1.getCurrentPosition() - d_RstartPoint) / ticksPerInch;
        LcuurentPosition = (ldrive1.getCurrentPosition() - d_RstartPoint) / ticksPerInch;

        dRPID.setSensorValue(RcuurentPosition);
        dLPID.setSensorValue(LcuurentPosition);

        dRPID.calculate();
        dLPID.calculate();

        if (dRPID.calculate() == minPower && accelerate > dRPID.calculate()) {
            d_Rpower = accelerate;
            accelerate -= 0.05;
        }
        if (dLPID.calculate() == minPower && accelerate > dLPID.calculate()) {
            d_Lpower = accelerate;
            accelerate -= 0.05;
        }


        while (dRPID.getError() < 0 && dLPID.getError() < 0 && opModeIsActive()) {

            RcuurentPosition = (rdrive1.getCurrentPosition()) / ticksPerInch;
            dRPID.setSensorValue(RcuurentPosition);

            d_Rpower = dRPID.calculate();


            LcuurentPosition = (ldrive1.getCurrentPosition()) / ticksPerInch;
            dLPID.setSensorValue(LcuurentPosition);

            d_Lpower = dLPID.calculate();

            aPID.setSensorValue(getAngle());
            coraction = aPID.calculate();

            if (dRPID.calculate() == maxPower && accelerate < dRPID.calculate()) {
                d_Rpower = accelerate;
                accelerate += 0.05;
            }
            if (dLPID.calculate() == maxPower && accelerate < dLPID.calculate()) {
                d_Lpower = accelerate;
                accelerate += 0.05;
            }

            telemetry.update();

            // set power levels.


            ldrive1.setPower(d_Lpower - coraction);
            ldrive2.setPower(d_Lpower - coraction);
            rdrive1.setPower(d_Rpower + coraction);
            rdrive2.setPower(d_Rpower + coraction);

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

    private void slideInches(double inches, double minPower, double maxPower) {


        sPID.setOutputRange(minPower, maxPower);


        sPID.setSetPoint(inches);


        s_startPoint = slide1.getCurrentPosition();


        ScPID.setSetPoint(0);
        ScPID.setOutputRange(-0.07, 0.07);

        ScurrentPosition = (slide1.getCurrentPosition() - s_startPoint) / SticksPerInch;


        sPID.setSensorValue(ScurrentPosition);


        sPID.calculate();


        while (sPID.getError() > 0 && opModeIsActive()) {

            ScurrentPosition = (slide1.getCurrentPosition() - s_startPoint) / SticksPerInch;
            sPID.setSensorValue(ScurrentPosition);

            d_Spower = sPID.calculate();


            ScPID.setSensorValue(getAngle());
            Scoraction = ScPID.calculate();


            telemetry.update();

            // set power levels.


            slide1.setPower(d_Spower);

            ldrive1.setPower(-coraction);
            ldrive2.setPower(-coraction);
            rdrive1.setPower(coraction);
            rdrive2.setPower(coraction);


            /*if (d_Rpower <0.06 && opModeIsActive()){
                dRPID.setSensorValue(inches);
                dRPID.calculate();
            }
            if (d_Lpower <0.06 && opModeIsActive()){
                dLPID.setSensorValue(inches);
                dLPID.calculate();
            }*/


            sleep(15);

            telemetry.addData("left position", ldrive2.getCurrentPosition());
            telemetry.addData("right position", rdrive2.getCurrentPosition());
            telemetry.addData("left power", ldrive2.getPower());
            telemetry.addData("right power", rdrive2.getPower());
        }


        slide1.setPower(0);
        ldrive1.setPower(0);
        ldrive2.setPower(0);
        rdrive1.setPower(0);
        rdrive2.setPower(0);


    }

    private void elevatorHight(double ticks) {
        ePID.setSensorValue(elevator.getCurrentPosition());
        ePID.setSetPoint(ticks);
        ePID.setOutputRange(-0.7, 0.7);
        while (ePID.getError() != 0) {
            elevator.setPower(ePID.calculate());
        }
    }

    private void setElevatorPosition(int ep) {
        double ticks;

        if (ep == 1) ticks = 0;
        else if (ep == 2) ticks = -1542;
        else if (ep == 3) ticks = -2717;
        else if (ep == 4) ticks = -4335;
        else ticks = 0;

        elevatorHight(ticks);
    }


}