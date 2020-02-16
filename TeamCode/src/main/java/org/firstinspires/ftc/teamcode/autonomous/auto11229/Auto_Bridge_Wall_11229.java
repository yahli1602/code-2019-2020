package org.firstinspires.ftc.teamcode.autonomous.auto11229;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.imageProsessing.TensorFlow;

import java.util.List;

@Autonomous(name = "Auto 11229 blue bridge wall", group = "11229 Stone")

public class Auto_Bridge_Wall_11229 extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotor lDrive1, lDrive2, rDrive1, rDrive2, slide1, elevator;
    BNO055IMU imu;
    Servo bazim;


    Orientation lastAngles = new Orientation();
    double globalAngle, rotation;
    boolean aButton, bButton, touched;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    PIDcon dRPID = new PIDcon();
    PIDcon dLPID = new PIDcon();
    PIDcon RLCPID = new PIDcon();
    PIDcon pidRotate = new PIDcon();
    PIDcon aPID = new PIDcon();
    PIDcon SaPID = new PIDcon();
    PIDcon sPID = new PIDcon();
    PIDcon ScPID = new PIDcon();

    TensorFlow TF = new TensorFlow();

    PIDcon a_pidDrive = new PIDcon();
    double d_error = 0;
    double d_prevError = 0;
    double d_RstartPoint = 0;
    double d_LstartPoint = 0;
    double a_startPoint = 0;
    double s_startPoint = 0;
    double sc_startPoint = 0;
    int y = 0;
    double coraction;
    double Scoraction;
    double RcuurentPosition = 0;
    double LcuurentPosition = 0;
    double ScurrentPosition = 0;


    int skystonePostion;
    int Stone1Postion;
    int Stone2Postion;
    int seeSkystone = 0;
    int seeStone1 = 0;
    int seeStone2 = 0;
    boolean canSeeSkystone = false;

    double d_Rpower = 0;
    double d_Lpower = 0;

    int h = 0;
    int f = 0;
    double integral = 0;
    double derivative = 0;

    private final double perimeter = 4 * Math.PI;
    private final double ticksPerRevolution = 28;

    private final double ticksPerSpin = ticksPerRevolution * 26.6666666666666666666666666666666666666666666666666666666666666666;
    private final double ticksPerInch = 1 / perimeter * ticksPerSpin;

    private final double SticksPerSpin = ticksPerRevolution * 40;
    private final double SticksPerInch = 1 / perimeter * SticksPerSpin;

    // called when init button is  pressed.


    @Override
    public void runOpMode() throws InterruptedException {


        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide1 = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        bazim = hardwareMap.get(Servo.class, "bazim");


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
        pidRotate.PIDcon(0.01, 0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.


        dRPID.PIDcon(0.05, 0, 0);
        dLPID.PIDcon(0.05, 0, 0);

        RLCPID.PIDcon(0.02, 0, 0);

        sPID.PIDcon(0.14, 0, 0.15);
        ScPID.PIDcon(0.01, 0, 0.1);
        SaPID.PIDcon(0.025, 0, 0);

        aPID.PIDcon(0.012, 0, 0.005);


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


        // Set up parameters for driving in a straight line.


        // drive until end of period.

        f = 0;
        int t = 0;
        while (opModeIsActive() && f == 0) {
            //sleep(20000);
            driveInches(60, -0.4, 0.4);
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


    private void rotate(int degrees, double power, boolean reset) {
        // restart imu angle tracking.
        if (reset) {
            resetAngle();
        }


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
                lDrive1.setPower(-power);
                lDrive2.setPower(-power);
                rDrive1.setPower(power);
                rDrive2.setPower(power);

            } while (opModeIsActive() && (Math.abs(getAngle()) < (Math.abs(degrees) - 3)));
        } else
            do {
                pidRotate.setSensorValue(getAngle());
                power = pidRotate.calculate();
                lDrive1.setPower(-power);
                lDrive2.setPower(-power);
                rDrive1.setPower(power);
                rDrive2.setPower(power);


            } while (opModeIsActive() && (Math.abs(getAngle()) < (Math.abs(degrees) - 3)));


        rDrive1.setPower(0);
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);


        rotation = getAngle();


        sleep(500);


        resetAngle();
    }

    private void zRotate(double power) {
        // restart imu angle tracking.


        // if degrees > 359 we cap at 359 with same sign as original degrees.


        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetPoint(0);
        pidRotate.setOutputRange(0, power);


        pidRotate.setSensorValue(getAngle());
        pidRotate.calculate();

        if (getAngle() < 0) {

            do {
                pidRotate.setSensorValue(getAngle());
                power = pidRotate.calculate();
                lDrive1.setPower(-power);
                lDrive2.setPower(-power);
                rDrive1.setPower(power);
                rDrive2.setPower(power);

            } while (opModeIsActive() && (Math.abs(getAngle()) < (0 - 3)));
        } else
            do {
                pidRotate.setSensorValue(getAngle());
                power = pidRotate.calculate();
                lDrive1.setPower(-power);
                lDrive2.setPower(-power);
                rDrive1.setPower(power);
                rDrive2.setPower(power);


            } while (opModeIsActive() && (Math.abs(getAngle()) < (0 - 3)));


        rDrive1.setPower(0);
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);


        rotation = getAngle();


        sleep(500);


    }

    private void driveInches(double inches, double minimumP, double maximumP) {
        if (inches > 0) {
            forwardInches(inches, minimumP, maximumP);
        } else if (inches < 0) {
            backInches(inches, minimumP, maximumP);
        }
    }

    private void forwardInches(double inches, double minimumP, double maximumP) {

        double d_Rpower = 0;
        double d_Lpower = 0;
        double Spower = 0;

        dLPID.reset();
        dRPID.reset();
        RLCPID.reset();


        double exelerate = 0.05;


        RcuurentPosition = 0;
        LcuurentPosition = 0;

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
        aPID.setOutputRange(-0.08, 0.08);

        RLCPID.setSetPoint(0);
        RLCPID.setOutputRange(-0.2, 0.2);


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
            dLPID.setSensorValue(LcuurentPosition);
            d_Lpower = dLPID.calculate();


            telemetry.addData("first coraction", coraction);
            aPID.setSensorValue(getAngle());
            coraction = aPID.calculate();

            RLCPID.setSensorValue(-slide1.getCurrentPosition() / SticksPerInch);
            Spower = RLCPID.calculate();


            telemetry.addData("second coraction", coraction);
            telemetry.addData("lpower", d_Lpower);
            telemetry.addData("rdrive", d_Rpower);


            // set power levels.`

            if (exelerate < maximumP && opModeIsActive()) {
                d_Lpower = exelerate;
                d_Rpower = exelerate;
                exelerate = exelerate + 0.05;
            }

            lDrive1.setPower(d_Lpower - coraction);
            lDrive2.setPower(d_Lpower - coraction);

            rDrive1.setPower(d_Rpower + coraction);
            rDrive2.setPower(d_Rpower + coraction);
            //if (RcuurentPosition < inches * 0.9) slide1.setPower(Spower);

            //if (RcuurentPosition < inches * 0.9) slide1.setPower(Spower);


            telemetry.addData("rP", rDrive1.getCurrentPosition());
            telemetry.addData("lP", lDrive1.getCurrentPosition());
            telemetry.addData("rPow", rDrive1.getPower());
            telemetry.addData("lPow", lDrive1.getPower());
            telemetry.update();

            sleep(5);


        }


        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        telemetry.addData("motors", "off");


    }

    private void backInches(double inches, double minPower, double maxPower) {

        d_Rpower = 0;
        d_Lpower = 0;

        dLPID.reset();
        dRPID.reset();


        RcuurentPosition = 0;
        LcuurentPosition = 0;


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


        dRPID.setOutputRange(minPower, maxPower);
        dLPID.setOutputRange(minPower, maxPower);

        dRPID.setSetPoint(inches);
        dLPID.setSetPoint(inches);


        d_RstartPoint = rDrive1.getCurrentPosition();
        d_LstartPoint = lDrive1.getCurrentPosition();


        aPID.setSetPoint(0);
        aPID.setOutputRange(-0.08, 0.08);

        RcuurentPosition = (rDrive1.getCurrentPosition() - d_RstartPoint) / ticksPerInch;
        LcuurentPosition = (lDrive1.getCurrentPosition() - d_RstartPoint) / ticksPerInch;

        dRPID.setSensorValue(RcuurentPosition);
        dLPID.setSensorValue(LcuurentPosition);

        dRPID.calculate();
        dLPID.calculate();


        while (dRPID.getError() < 0 && dLPID.getError() < 0 && opModeIsActive()) {

            RcuurentPosition = (rDrive1.getCurrentPosition()) / ticksPerInch;
            dRPID.setSensorValue(RcuurentPosition);

            d_Rpower = dRPID.calculate();


            LcuurentPosition = (lDrive1.getCurrentPosition()) / ticksPerInch;
            dLPID.setSensorValue(LcuurentPosition);

            d_Lpower = dLPID.calculate();

            aPID.setSensorValue(getAngle());
            coraction = aPID.calculate();


            telemetry.update();

            // set power levels.


            lDrive1.setPower(d_Lpower - coraction);
            lDrive2.setPower(d_Lpower - coraction);
            rDrive1.setPower(d_Rpower + coraction);
            rDrive2.setPower(d_Rpower + coraction);


            sleep(5);

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


    private void slideInches(double inches, double minimumPs, double maximumPs) {
        if (inches > 0) {
            forwardSlideInches(inches, minimumPs, maximumPs);
        } else if (inches < 0) {
            backSlideInches(inches, minimumPs, maximumPs);
        }
    }

    private void forwardSlideInches(double inches, double minimumPs, double maximumPs) {

        double d_Spower = 0;
        double Rpower = 0;
        double Lpower = 0;


        h = 0;

        sPID.reset();
        ScPID.reset();
        SaPID.reset();


        ScurrentPosition = 0;


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
        ScPID.setOutputRange(-0.08, 0.08);

        SaPID.setSetPoint(0);
        SaPID.setOutputRange(-0.04, 0.04);


        s_startPoint = slide1.getCurrentPosition();

        sc_startPoint = getAngle();


        ScurrentPosition = slide1.getCurrentPosition() / SticksPerInch;
        sPID.setSensorValue(ScurrentPosition);
        sPID.calculate();


        while (sPID.getError() > 0 && opModeIsActive()) {


            ScurrentPosition = (slide1.getCurrentPosition()) / SticksPerInch;
            sPID.setSensorValue(ScurrentPosition);
            d_Spower = sPID.calculate();

            RcuurentPosition = rDrive1.getCurrentPosition() / ticksPerInch;
            SaPID.setSensorValue(RcuurentPosition);
            Rpower = SaPID.calculate();
            h++;

            LcuurentPosition = lDrive1.getCurrentPosition() / ticksPerInch;
            SaPID.setSensorValue(LcuurentPosition);
            Lpower = SaPID.calculate();


            ScPID.setSensorValue(getAngle());
            Scoraction = ScPID.calculate();


            // set power levels.`
            slide1.setPower(d_Spower);
            lDrive1.setPower(Lpower - Scoraction);
            lDrive2.setPower(Lpower - Scoraction);
            rDrive1.setPower(Rpower + Scoraction);
            rDrive2.setPower(Rpower + Scoraction);


            telemetry.addData("sError", sPID.getError());
            telemetry.addData("Sc", slide1.getCurrentPosition());


            telemetry.update();

            sleep(5);


        }


        slide1.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);

        telemetry.addData("motors", "off");


    }

    private void backSlideInches(double inches, double minimumPs, double maximumPs) {

        double d_Spower = 0;
        double Rpower = 0;
        double Lpower = 0;


        h = 0;

        sPID.reset();
        ScPID.reset();
        SaPID.reset();


        ScurrentPosition = 0;


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
        ScPID.setOutputRange(-0.05, 0.05);

        SaPID.setSetPoint(0);
        SaPID.setOutputRange(-0.04, 0.04);


        s_startPoint = -slide1.getCurrentPosition();

        sc_startPoint = getAngle();


        ScurrentPosition = slide1.getCurrentPosition() / SticksPerInch;
        sPID.setSensorValue(ScurrentPosition);
        sPID.calculate();


        while (sPID.getError() < 0 && opModeIsActive()) {


            ScurrentPosition = (slide1.getCurrentPosition() - s_startPoint) / SticksPerInch;
            sPID.setSensorValue(ScurrentPosition);
            d_Spower = sPID.calculate();

            RcuurentPosition = rDrive1.getCurrentPosition() / ticksPerInch;
            SaPID.setSensorValue(RcuurentPosition);
            Rpower = SaPID.calculate();

            LcuurentPosition = lDrive1.getCurrentPosition() / ticksPerInch;
            SaPID.setSensorValue(LcuurentPosition);
            Lpower = SaPID.calculate();


            ScPID.setSensorValue(getAngle());
            Scoraction = ScPID.calculate();


            // set power levels.`
            slide1.setPower(d_Spower);
            lDrive1.setPower(Lpower - Scoraction);
            lDrive2.setPower(Lpower - Scoraction);
            rDrive1.setPower(Rpower + Scoraction);
            rDrive2.setPower(Rpower + Scoraction);


            telemetry.addData("slide error", sPID.getError());
            telemetry.addData("slide CP", ScurrentPosition);
            telemetry.addData("slide pow", slide1.getPower());

            telemetry.update();

            sleep(5);


        }


        slide1.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);

        telemetry.addData("motors", "off");


    }

    public void teleop_11226_A() {
        ElapsedTime time = new ElapsedTime();

        boolean pinch = false;
        boolean pushCube = false;
        //driving motors
        DcMotor rDrive1 = null;
        DcMotor rDrive2 = null;
        DcMotor lDrive1 = null;
        DcMotor lDrive2 = null;
        DcMotor slide = null;
        //elevator
        DcMotor elevator = null;
        //collection
        DcMotor collectRight = null;
        DcMotor collectLeft = null;
        Servo pushLeft = null;
        Servo pushRight = null;
        //Pinch
        CRServo hold = null;
        CRServo turnHold = null;
        Servo bazim = null;
        //moving Foundation
        Servo grabber = null;
        TouchSensor cubeIn = null;
        boolean Fast = true;
        PIDcon ePID = new PIDcon();
        int elevatorPosition;
        // problem fixing
        boolean hold180 = false;
        double fix = 0;
        boolean pinchDown = true;

        boolean canTimerWork = true;
        double lastEp;

        boolean pinchIn = true;
        //Time timer1 = new Time(2000);


    /*class turnHP extends TimerTask {
        @Override
        public void run() {
            turnHold.setPower(1);
        }
    }


    class turnHM extends TimerTask {
        @Override
        public void run() {
            turnHold.setPower(1);
            canTimerWork = true;
        }
    }


    turnHP turnHPlus = new turnHP();
    turnHM turnHMinus = new turnHM();


    Timer timer = new Timer();*/


        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        turnHold = hardwareMap.get(CRServo.class, "turnHold");
        hold = hardwareMap.get(CRServo.class, "hold");
        pushLeft = hardwareMap.get(Servo.class, "pushLeft");
        pushRight = hardwareMap.get(Servo.class, "pushRight");
        collectRight = hardwareMap.get(DcMotor.class, "collectRight");
        collectLeft = hardwareMap.get(DcMotor.class, "collectLeft");
        bazim = hardwareMap.get(Servo.class, "bazim");

        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setDirection(DcMotor.Direction.FORWARD);
        lDrive2.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        pushLeft.setDirection(Servo.Direction.FORWARD);
        pushRight.setDirection(Servo.Direction.REVERSE);
        collectLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        collectRight.setDirection(DcMotorSimple.Direction.REVERSE);
        /*pushLeft.setPosition(0);
        pushRight.setPosition(0);*/
        bazim.setDirection(Servo.Direction.FORWARD);

        bazim.setPosition(0);
        pushLeft.setPosition(0);
        pushRight.setPosition(0);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();


//
        rDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ePID.PIDcon(0.0001, 0, 0);

        while (opModeIsActive()) {

            //drive

            if (Fast) {
                if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                    rDrive1.setPower(gamepad1.left_stick_y - fix);
                    rDrive2.setPower(gamepad1.left_stick_y - fix);
                    lDrive1.setPower(gamepad1.left_stick_y - fix);
                    lDrive2.setPower(gamepad1.left_stick_y - fix);
                } else if (gamepad1.left_trigger > 0.2) {
                    rDrive1.setPower(-gamepad1.left_trigger - fix);
                    rDrive2.setPower(-gamepad1.left_trigger - fix);
                    lDrive1.setPower(gamepad1.left_trigger + fix);
                    lDrive2.setPower(gamepad1.left_trigger + fix);
                } else if (gamepad1.right_trigger > 0.2) {
                    rDrive1.setPower(gamepad1.right_trigger + fix);
                    rDrive2.setPower(gamepad1.right_trigger + fix);
                    lDrive1.setPower(-gamepad1.right_trigger - fix);
                    lDrive2.setPower(-gamepad1.right_trigger - fix);
                } else {
                    rDrive1.setPower(0);
                    rDrive2.setPower(0);
                    lDrive1.setPower(0);
                    lDrive2.setPower(0);
                }
            } else {
                if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                    rDrive1.setPower(gamepad1.left_stick_y / 2 - fix);
                    rDrive2.setPower(gamepad1.left_stick_y / 2 - fix);
                    lDrive1.setPower(gamepad1.left_stick_y / 2 - fix);
                    lDrive2.setPower(gamepad1.left_stick_y / 2 - fix);
                } else if (gamepad1.left_trigger > 0.2) {
                    rDrive1.setPower(gamepad1.left_trigger / 2 - fix);
                    rDrive2.setPower(gamepad1.left_trigger / 2 - fix);
                    lDrive1.setPower(-gamepad1.left_trigger / 2 + fix);
                    lDrive2.setPower(-gamepad1.left_trigger / 2 + fix);
                } else if (gamepad1.right_trigger > 0.2) {
                    rDrive1.setPower(-gamepad1.right_trigger / 2 + fix);
                    rDrive2.setPower(-gamepad1.right_trigger / 2 + fix);
                    lDrive1.setPower(gamepad1.right_trigger / 2 - fix);
                    lDrive2.setPower(gamepad1.right_trigger / 2 - fix);
                } else {
                    rDrive1.setPower(0);
                    rDrive2.setPower(0);
                    lDrive1.setPower(0);
                    lDrive2.setPower(0);
                }
            }


            while (lDrive1.isBusy()) {
                if (Fast) {
                    if ((gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) && lDrive2.getPower() != gamepad1.left_stick_y) {
                        fix = lDrive2.getPower() - gamepad1.left_stick_y;
                    } else if (gamepad1.left_trigger > 0.2 && rDrive2.getPower() != gamepad1.left_trigger) {
                        fix = rDrive2.getPower() - gamepad1.left_trigger;
                    } else if (gamepad1.right_trigger > 0.2 && lDrive2.getPower() != gamepad1.right_trigger) {
                        fix = lDrive2.getPower() - gamepad1.right_trigger;
                    } else if (lDrive2.isBusy()) {
                        fix = lDrive2.getPower();
                    }
                } else {
                    if ((gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) && lDrive2.getPower() * 2 != gamepad1.left_stick_y) {
                        fix = lDrive2.getPower() - gamepad1.left_stick_y;
                    } else if (gamepad1.left_trigger > 0.2 && rDrive2.getPower() * 2 != gamepad1.left_trigger) {
                        fix = rDrive2.getPower() - gamepad1.left_trigger;
                    } else if (gamepad1.right_trigger > 0.2 && lDrive2.getPower() * 2 != gamepad1.right_trigger) {
                        fix = lDrive2.getPower() - gamepad1.right_trigger;
                    } else if (lDrive2.isBusy()) {
                        fix = lDrive2.getPower();
                    }
                }
            }


            if (gamepad1.a) {
                Fast = true;
            }
            if (gamepad1.b) {
                Fast = false;
            }


            if (gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < 0.2) {
                slide.setPower(gamepad1.right_stick_x);
            } else {
                slide.setPower(0);
            }


            if (gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            } else {
                elevator.setPower(0);
            }


            if (gamepad2.a) {
                pushLeft.setPosition(1);
                pushRight.setPosition(1);
            } else if (gamepad2.b) {
                pushLeft.setPosition(0);
                pushRight.setPosition(0);
            } else {
                pushRight.setPosition(0.23);
                pushLeft.setPosition(0.23);
            }


            if (gamepad2.x) {
                pinchDown = true;
                hold.setPower(-1);
            } else if (gamepad2.y) {
                pinchDown = false;
                hold.setPower(1);
            } else if (pinchDown) {
                hold.setPower(0);
            } else {
                hold.setPower(1);
            }


            if (gamepad2.right_stick_x > 0.2) {
                turnHold.setPower(0.3);
            } else if (gamepad2.right_stick_x < -0.2) {
                turnHold.setPower(-0.3);
            } else {
                turnHold.setPower(0);
            }

            if (gamepad2.right_stick_button) {
                elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            if (gamepad2.right_trigger > 0) {
                collectLeft.setPower(1);
                collectRight.setPower(1);
            } else if (gamepad2.left_trigger > 0) {
                collectLeft.setPower(-1);
                collectRight.setPower(-1);
            } else {
                collectRight.setPower(0);
                collectLeft.setPower(0);
            }

            if (gamepad1.x) {
                y++;
            }



            /*telemetry.addData("dpad up", gamepad2.dpad_up);
            telemetry.addData("dpad down", gamepad2.dpad_down);
            telemetry.addData("dpad right", gamepad2.dpad_right);
            telemetry.addData("dpad left", gamepad2.dpad_left);
            //telemetry.addData("timer",timer1.getTime());
            telemetry.update();*/


            telemetry.addData("elevator ticks", elevator.getCurrentPosition());
            telemetry.update();




            /*if(gamepad2.b){
                setElevatorPosition(2);
                turnHold.setPower(1);
                sleep(2000);
            }*/

            //turn collection


            // Automations


        }

    }


    private void correctAngle() {
        zRotate(0.3);

    }


}