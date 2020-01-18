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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "PID drive 11226", group = "PID")

public class PIDdrive_11226 extends LinearOpMode {
    DcMotor ldrive1, ldrive2, rdrive1, rdrive2, slide1, elevator;
    CRServo hold;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .07, correction, rotation;
    boolean aButton, bButton, touched;
    PIDcon dRPID = new PIDcon();
    PIDcon dLPID = new PIDcon();
    PIDcon pidRotate = new PIDcon();
    PIDcon aPID = new PIDcon();
    PIDcon ePID = new PIDcon();
    PIDcon sPID = new PIDcon();
    PIDcon ScPID = new PIDcon();
    double d_error = 0;
    double                  s_startPoint = 0;
    double                  sc_startPoint = 0;
    double                  Scoraction;
    double ScurrentPosition = 0;
    double RcuurentPosition = 0;
    double LcuurentPosition = 0;
    double d_prevError = 0;
    double d_RstartPoint = 0;
    double d_LstartPoint = 0;


    double integral = 0;
    double derivative = 0;
    double  d_Rpower = 0;
    double  d_Lpower = 0;
    double  d_Spower = 0;
    double coraction = 0;


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
        slide1 = hardwareMap.get(DcMotor.class, "slide");

        hold = hardwareMap.get(CRServo.class, "hold");

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        /*collectRight = hardwareMap.get(Servo.class, "collect right");
        collectLeft = hardwareMap.get(Servo.class, "collect left");
        cubeIn = hardwareMap.get(TouchSensor.class, "cube in");*/

        ePID.PIDcon(0.1,0,0);

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
        slide1.setDirection(DcMotor.Direction.FORWARD);

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
        pidRotate.PIDcon(0.01,0,0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.


        dRPID.PIDcon(0.07, 0, 0.12);
        dLPID.PIDcon(0.07, 0, 0.12);

        aPID.PIDcon(0.2,0,0.1);

        sPID.PIDcon(0.1,0,0);
        ScPID.PIDcon(0,0,0);

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





            driveInches(48,-0.6,0.6);

            q++;

        }
        // Use PID with imu input to drive in a straight line.

    }


    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

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
        pidRotate.setSetPoint(degrees);
        pidRotate.setOutputRange(0, power);



        pidRotate.setSensorValue(getAngle());
        pidRotate.calculate();

        if (degrees < 0) {

            do {
                pidRotate.setSensorValue(getAngle());
                power = pidRotate.calculate();
                ldrive1.setPower(-power);
                ldrive2.setPower(-power);
                rdrive1.setPower(power);
                rdrive2.setPower(power);

            } while (opModeIsActive() && pidRotate.getError() != 0);
        } else
            do {
                pidRotate.setSensorValue(getAngle());
                power = pidRotate.calculate();
                ldrive1.setPower(-power);
                ldrive2.setPower(-power);
                rdrive1.setPower(power);
                rdrive2.setPower(power);


            } while (opModeIsActive() && pidRotate.getError() != 0);


        rdrive1.setPower(0);
        rdrive2.setPower(0);
        ldrive1.setPower(0);
        ldrive2.setPower(0);


        rotation = getAngle();


        sleep(500);


        resetAngle();
    }



    private void driveInches(double inches ,double minimumP ,double maximumP){
        if (inches > 0){
            forwardInches(inches,minimumP,maximumP);
        }else if (inches < 0){
            backInches(inches,minimumP,maximumP);
        }
    }

    private void forwardInches(double inches, double minPower, double maxPower) {

        d_Rpower = 0;
        d_Lpower = 0;

        dRPID.setOutputRange(minPower,maxPower);
        dLPID.setOutputRange(minPower,maxPower);

        dRPID.setSetPoint(inches);
        dLPID.setSetPoint(inches);


        d_RstartPoint = rdrive1.getCurrentPosition();
        d_LstartPoint = ldrive1.getCurrentPosition();


        aPID.setSetPoint(0);
        aPID.setOutputRange(-0.07,0.07);

        RcuurentPosition = (rdrive1.getCurrentPosition() - d_RstartPoint) / ticksPerInch;
        LcuurentPosition = (ldrive1.getCurrentPosition() - d_RstartPoint) / ticksPerInch;

        dRPID.setSensorValue(RcuurentPosition);
        dLPID.setSensorValue(LcuurentPosition);

        dRPID.calculate();
        dLPID.calculate();



        while (dRPID.getError() > 0 && dLPID.getError() > 0 && opModeIsActive()) {

            RcuurentPosition = (rdrive1.getCurrentPosition()) / ticksPerInch;
            dRPID.setSensorValue(RcuurentPosition);

            d_Rpower = dRPID.calculate();


            LcuurentPosition = (ldrive1.getCurrentPosition()) / ticksPerInch;
            dLPID.setSensorValue(LcuurentPosition);

            d_Lpower = dLPID.calculate();

            aPID.setSensorValue(getAngle());
            coraction = aPID.calculate();










            telemetry.update();

            // set power levels.








            ldrive1.setPower(d_Lpower - coraction);
            ldrive2.setPower(d_Lpower - coraction);
            rdrive1.setPower(d_Rpower + coraction);
            rdrive2.setPower(d_Rpower + coraction);

            /*if (d_Rpower <0.06 && opModeIsActive()){
                dRPID.setSensorValue(inches);
                dRPID.calculate();
            }
            if (d_Lpower <0.06 && opModeIsActive()){
                dLPID.setSensorValue(inches);
                dLPID.calculate();
            }*/



            sleep(50);

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

    private void backInches(double inches, double minPower, double maxPower) {

        d_Rpower = 0;
        d_Lpower = 0;

        dRPID.setOutputRange(minPower,maxPower);
        dLPID.setOutputRange(minPower,maxPower);

        dRPID.setSetPoint(inches);
        dLPID.setSetPoint(inches);


        d_RstartPoint = rdrive1.getCurrentPosition();
        d_LstartPoint = ldrive1.getCurrentPosition();


        aPID.setSetPoint(0);
        aPID.setOutputRange(-0.07,0.07);

        RcuurentPosition = (rdrive1.getCurrentPosition() - d_RstartPoint) / ticksPerInch;
        LcuurentPosition = (ldrive1.getCurrentPosition() - d_RstartPoint) / ticksPerInch;

        dRPID.setSensorValue(RcuurentPosition);
        dLPID.setSensorValue(LcuurentPosition);

        dRPID.calculate();
        dLPID.calculate();



        while (dRPID.getError() < 0 && dLPID.getError() < 0 && opModeIsActive()) {

            RcuurentPosition = (rdrive1.getCurrentPosition()) / ticksPerInch;
            dRPID.setSensorValue(RcuurentPosition);

            d_Rpower = dRPID.calculate();


            LcuurentPosition = (ldrive1.getCurrentPosition()) / ticksPerInch;
            dLPID.setSensorValue(LcuurentPosition);

            d_Lpower = dLPID.calculate();

            aPID.setSensorValue(getAngle());
            coraction = aPID.calculate();










            telemetry.update();

            // set power levels.








            ldrive1.setPower(d_Lpower - coraction);
            ldrive2.setPower(d_Lpower - coraction);
            rdrive1.setPower(d_Rpower + coraction);
            rdrive2.setPower(d_Rpower + coraction);





            sleep(50);

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



    private void forwardSlideInches(double inches, double minPower, double maxPower) {



        sPID.setOutputRange(minPower,maxPower);


        sPID.setSetPoint(inches);



        s_startPoint = slide1.getCurrentPosition();



        ScPID.setSetPoint(0);
        ScPID.setOutputRange(-0.07,0.07);

        ScurrentPosition = (slide1.getCurrentPosition() - s_startPoint) / ticksPerInch;


        sPID.setSensorValue(ScurrentPosition);


        sPID.calculate();




        while (sPID.getError() > 0 && opModeIsActive()) {

            ScurrentPosition = (slide1.getCurrentPosition() - s_startPoint) / ticksPerInch;
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






            telemetry.addData("ScurrentP", ScurrentPosition);
            telemetry.addData("S error", sPID.getError());

            telemetry.update();

            sleep(50);


        }


        slide1.setPower(0);
        ldrive1.setPower(0);
        ldrive2.setPower(0);
        rdrive1.setPower(0);
        rdrive2.setPower(0);


    }



    private void elevatorHight(double ticks){
        ePID.setSensorValue(elevator.getCurrentPosition());
        ePID.setSetPoint(ticks);
        ePID.setOutputRange(-0.7,0.7);
        while(ePID.getError() != 0){
            elevator.setPower(ePID.calculate());
        }
    }

    private void setElevatorPosition(int ep){
        double ticks;

        if (ep == 1) ticks = 0;
        else if (ep == 2) ticks = -1542;
        else if (ep == 3) ticks = -2717;
        else if (ep == 4) ticks = -4335;
        else ticks = 0;

        elevatorHight(ticks);
    }




}