package org.firstinspires.ftc.teamcode.autonomous.auto11229;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.PIDController.PIDcon;
import org.firstinspires.ftc.teamcode.autonomous.imageProsessing.TensorFlow;

import java.util.List;
//
@Disabled
@Autonomous(name="BlueStone 11229 2", group="Stone")

public class Blue2Stone_11229_2 extends LinearOpMode
{
    DcMotor                 lDrive1,lDrive2,rDrive1,rDrive2,slide1,elevator;
    BNO055IMU               imu;
    Servo                   bazim;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction, rotation;
    boolean                 aButton, bButton, touched;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    PIDcon dRPID = new PIDcon();
    PIDcon dLPID = new PIDcon();
    PIDcon pidRotate = new PIDcon();
    PIDcon aPID = new PIDcon();
    PIDcon SaPID = new PIDcon();
    PIDcon sPID = new PIDcon();
    PIDcon ScPID = new PIDcon();

    TensorFlow TF = new TensorFlow();

    PIDcon a_pidDrive = new PIDcon();
    double                  d_error = 0;
    double                  d_prevError = 0;
    double                  d_RstartPoint = 0;
    double                  d_LstartPoint = 0;
    double                  a_startPoint = 0;
    double                  s_startPoint = 0;
    double                  sc_startPoint = 0;
    double                  coraction;
    double                  Scoraction;
    double RcuurentPosition = 0;
    double LcuurentPosition = 0;
    double ScurrentPosition = 0;

    private float skyStoneX = 0;
    private float Stone1X = 0;
    private float Stone2X = 0;
    int skystonePostion;
    int Stone1Postion;
    int Stone2Postion;
    int seeSkystone = 0;
    int seeStone1 = 0;
    int seeStone2 = 0;
    boolean canSeeSkystone = false;

    double  d_Rpower = 0;
    double  d_Lpower = 0;

    int h = 0;
    int f = 0;
    double integral = 0;
    double derivative = 0;

    private final double perimeter = 4 * Math.PI;
    private final double ticksPerRevolution = 28;
    private final double ticksPerSpin = ticksPerRevolution * 20;
    private final double ticksPerInch = 1 / perimeter* ticksPerSpin;

    private final double SticksPerSpin = ticksPerRevolution * 35;
    private final double SticksPerInch = 1 / perimeter* SticksPerSpin;

    // called when init button is  pressed.



    private static final String VUFORIA_KEY =
            "AVHZDTL/////AAABmQcZurBiA01smn3EpdcPCJpZqB8HZL60ujXKBU3ejemhikdsno1L3+7QKhYWSXEfUl5uWZxBqPJXl6Qj0AG3XKuq/jLKmyLJ67xHlYM/LoVKbxhjxGJJ5stO+21qtYET0KberI6XObNkTmskQ8kLQX7QwLhmllfyhu25bPFWwmVdnGq3jRAxoCNKP9ktqKkqp62Fl39qcvOwCOBPqG0uFMFHwVaNavRHS1f4fnuZXk4QqEDo5e2K9J/sCR/2BvvzdPV3QfTkUPNm/8dfW2nsxCM2E9rpj67CFq9fOAHjY+7tp4o2U/yJbxc5RBr5mZ9/CeQk7zfl9rQv7WrVWevfvHqvb2xMsoqVJGze9rE62AmI";

    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException
    {


        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide1 = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "teleop_11226_A");
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

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C 0port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);



        if (tfod != null) {
            tfod.activate();
        }



        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate.PIDcon(0.01,0.0018,0.135);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.


        dRPID.PIDcon(0.05,0,0);
        dLPID.PIDcon(0.05,0,0);

        sPID.PIDcon(0.005,0.0009,0.1);
        ScPID.PIDcon(0.02,0,0.07);
        SaPID.PIDcon(0.025,0,0);

        aPID.PIDcon(0.05,0,0);



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

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();

        }


        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // Set up parameters for driving in a straight line.





        // drive until end of period.

        f = 0;
        while (opModeIsActive() && f == 0)

        {
            slideInches(30,0.03,0.2);
            bazim.setPosition(0.60);
            sleep(400);
            slide1.setPower(-1);
            sleep(600);
            slide1.setPower(0);
            rotate(90,0.2,true);
            slideInches(60,0,0.5);
            bazim.setPosition(0);
            slide1.setPower(-1);
            sleep(950);
            slide1.setPower(0);
            sleep(50);
            lDrive1.setPower(0.6);
            lDrive2.setPower(0.6);
            rDrive1.setPower(0.6);
            rDrive2.setPower(0.6);
            sleep(640);
            lDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive1.setPower(0);
            rDrive2.setPower(0);
            elevator.setPower(1);
            sleep(400);
            elevator.setPower(-1);
            sleep(100);
            elevator.setPower(0);




            f++;

        }

        if (tfod != null) {
            tfod.shutdown();
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


    private void rotate(int degrees, double power,boolean reset) {
        // restart imu angle tracking.
        if (reset){
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

            } while (opModeIsActive() && (Math.abs(getAngle()) < (Math.abs(degrees) - 3 )));
        } else
            do {
                pidRotate.setSensorValue(getAngle());
                power = pidRotate.calculate();
                lDrive1.setPower(-power);
                lDrive2.setPower(-power);
                rDrive1.setPower(power);
                rDrive2.setPower(power);


            } while (opModeIsActive() && (Math.abs(getAngle()) < (Math.abs(degrees) - 3 )));


        rDrive1.setPower(0);
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);


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

    private void forwardInches(double inches ,double minimumP ,double maximumP) {

        double d_Rpower = 0;
        double d_Lpower = 0;

        dLPID.reset();
        dRPID.reset();
        resetAngle();

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
        dRPID.setOutputRange(minimumP , maximumP);

        dLPID.setSetPoint(inches);
        dLPID.setOutputRange(minimumP , maximumP);

        //aPID.setSetPoint(0);
        //aPID.setOutputRange(-0.07,0.07);


        d_RstartPoint = rDrive1.getCurrentPosition();

        d_LstartPoint = lDrive1.getCurrentPosition();

        a_startPoint = getAngle();



        RcuurentPosition = rDrive1.getCurrentPosition()/ ticksPerInch;
        dRPID.setSensorValue(RcuurentPosition);
        dRPID.calculate();

        LcuurentPosition = lDrive1.getCurrentPosition()/ ticksPerInch;
        dLPID.setSensorValue(LcuurentPosition);
        dLPID.calculate();


        while (dRPID.getError() > 0  && dLPID.getError() > 0 && opModeIsActive()) {



            RcuurentPosition = (rDrive1.getCurrentPosition() / ticksPerInch);
            dRPID.setSensorValue(RcuurentPosition);
            d_Rpower = dRPID.calculate();

            LcuurentPosition = (lDrive1.getCurrentPosition()/ ticksPerInch);
            dLPID.setSensorValue(LcuurentPosition);
            d_Lpower = dLPID.calculate();

            //aPID.setSensorValue(getAngle());
            coraction = checkDirection() ;//aPID.calculate();



            telemetry.update();

            // set power levels.`
            lDrive1.setPower(d_Lpower - coraction);
            lDrive2.setPower(d_Lpower - coraction);

            rDrive1.setPower(d_Rpower + coraction);
            rDrive2.setPower(d_Rpower + coraction);


            telemetry.addData("left power", lDrive2.getPower());
            telemetry.addData("right power", rDrive2.getPower());
            telemetry.addData("RError",dRPID.getError());
            telemetry.addData("LError",dLPID.getError());
            telemetry.addData("RcurrentP", RcuurentPosition);
            telemetry.addData("LcurrentP", LcuurentPosition);
            telemetry.update();

            sleep(15);



        }


        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        telemetry.addData("motors","off");


    }

    private void backInches(double inches, double minPower, double maxPower) {

        d_Rpower = 0;
        d_Lpower = 0;

        dLPID.reset();
        dRPID.reset();
        resetAngle();

        dRPID.setOutputRange(minPower,maxPower);
        dLPID.setOutputRange(minPower,maxPower);

        dRPID.setSetPoint(inches);
        dLPID.setSetPoint(inches);


        d_RstartPoint = rDrive1.getCurrentPosition();
        d_LstartPoint = lDrive1.getCurrentPosition();


        aPID.setSetPoint(0);
        aPID.setOutputRange(-0.07,0.07);

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





            sleep(50);

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



    private void slideInches(double inches ,double minimumPs ,double maximumPs){
        if (inches > 0){
            forwardSlideInches(inches,minimumPs,maximumPs);
        }else if (inches < 0){
            backSlideInches(inches,minimumPs,maximumPs);
        }
    }

    private void forwardSlideInches(double inches ,double minimumPs ,double maximumPs) {

        double d_Spower = 0;
        double Rpower = 0;
        double Lpower = 0;


        h = 0;

        sPID.reset();
        ScPID.reset();
        resetAngle();



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
        sPID.setOutputRange(minimumPs,maximumPs);



        ScPID.setSetPoint(0);
        ScPID.setOutputRange(-0.02,0.02);

        SaPID.setSetPoint(0);
        SaPID.setOutputRange(-0.04,0.04);





        s_startPoint = slide1.getCurrentPosition();

        sc_startPoint = getAngle();



        ScurrentPosition = -slide1.getCurrentPosition()/ SticksPerInch;
        sPID.setSensorValue(ScurrentPosition);
        sPID.calculate();




        while (sPID.getError() > 0 && opModeIsActive()) {



            ScurrentPosition = (-slide1.getCurrentPosition()  - s_startPoint)/ SticksPerInch;
            sPID.setSensorValue(ScurrentPosition);
            d_Spower = sPID.calculate();

            RcuurentPosition = rDrive1.getCurrentPosition()/ ticksPerInch;
            SaPID.setSensorValue(RcuurentPosition);
            Rpower = SaPID.calculate();

            LcuurentPosition = lDrive1.getCurrentPosition()/ ticksPerInch;
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

            sleep(15);




        }


        slide1.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);

        telemetry.addData("motors","off");


    }

    private void backSlideInches(double inches ,double minimumPs ,double maximumPs) {

        double d_Spower = 0;
        double Rpower = 0;
        double Lpower = 0;


        h = 0;

        sPID.reset();
        ScPID.reset();
        resetAngle();



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
        sPID.setOutputRange(minimumPs,maximumPs);


        ScPID.setSetPoint(0);
        ScPID.setOutputRange(-0.02,0.02);

        SaPID.setSetPoint(0);
        SaPID.setOutputRange(-0.04,0.04);



        s_startPoint = -slide1.getCurrentPosition();

        sc_startPoint = getAngle();



        ScurrentPosition = -slide1.getCurrentPosition()/ SticksPerInch;
        sPID.setSensorValue(ScurrentPosition);
        sPID.calculate();




        while (sPID.getError() < 0 && opModeIsActive()) {



            ScurrentPosition = (-slide1.getCurrentPosition()  - s_startPoint)/ SticksPerInch;
            sPID.setSensorValue(ScurrentPosition);
            d_Spower = sPID.calculate();

            RcuurentPosition = rDrive1.getCurrentPosition()/ ticksPerInch;
            SaPID.setSensorValue(RcuurentPosition);
            Rpower = SaPID.calculate();

            LcuurentPosition = lDrive1.getCurrentPosition()/ ticksPerInch;
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

            sleep(50);




        }


        slide1.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);

        telemetry.addData("motors","off");


    }

    private double checkDirection(){

        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 0.0035;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        if (correction > 0.07) correction = 0.07;

        if (correction < -0.07) correction = -0.07;

        return correction;
    }





    private int seeTwoObj(List<Recognition> Recognitions){

        if (Recognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT)){
            skyStoneX = Recognitions.get(0).getLeft();
            Stone1X = Recognitions.get(1).getLeft();
        }else if (Recognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT)){
            Stone1X = Recognitions.get(0).getLeft();
            skyStoneX = Recognitions.get(1).getLeft();
        }





        if (skyStoneX < Stone1X){
            skystonePostion = 1;
        }else if (skyStoneX > Stone1X) {
            skystonePostion = 3;
        }


        return skystonePostion;
    }

    private int seeThreeObj(List<Recognition> Recognitions){

        if (Recognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT)){
            skyStoneX = Recognitions.get(0).getLeft();
        }else if (Recognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT)){
            Stone1X = Recognitions.get(0).getLeft();
        }


        if (Recognitions.get(1).getLabel().equals(LABEL_SECOND_ELEMENT)){
            skyStoneX = Recognitions.get(1).getLeft();

        }else if (Recognitions.get(1).getLabel().equals(LABEL_FIRST_ELEMENT)) {

            if (Stone1X != 0) {

                Stone2X = Recognitions.get(1).getLeft();
            } else if (Stone1X == 0) {
                Stone1X = Recognitions.get(1).getLeft();
            }
        }

        if (Recognitions.get(2).getLabel().equals(LABEL_SECOND_ELEMENT)){
            skyStoneX = Recognitions.get(2).getLeft();
        }else if (Recognitions.get(2).getLabel().equals(LABEL_FIRST_ELEMENT)){
            Stone2X = Recognitions.get(2).getLeft();
        }


        if (skyStoneX < Stone1X && skyStoneX < Stone2X){
            skystonePostion = 1;
        }else if (skyStoneX > Stone1X && skyStoneX > Stone2X){
            skystonePostion = 3;
        }else if (skyStoneX > Stone1X && skyStoneX < Stone2X || skyStoneX < Stone1X && skyStoneX > Stone2X) {
            skystonePostion = 2;
        }

        return skystonePostion;
    }



    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        vuforia = ClassFactory.getInstance().createVuforia(parameters);


    }


    //init the TensorFlow
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.45;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }




}