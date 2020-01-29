
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@Disabled
@Autonomous(name = "Auto 11226 Blue Build Zone", group = "Autonomous")
public class Auto_11226_B_Blue_Build_Zone extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    ElapsedTime elapsedTime = new ElapsedTime();
    public DcMotor rdrive1 = null;
    public DcMotor rdrive2 = null;
    public DcMotor ldrive1 = null;
    public DcMotor ldrive2 = null;
    public DcMotor slide1 = null;
    public DcMotor slide2 = null;
    public DcMotor elevator = null;
    public CRServo hold = null;
    /*private Servo collectRight = null;
    private Servo collectLeft = null;
    private TouchSensor cubeIn = null;*/

    private double kp = 0.2;
    private double ks = 0.1;
    private double ki = 0.2;
    private double kd = 0.1;
    private double uT = 1;
    private double pU;
    private double errorT;
    private double currentPosition;
    private double lastError;
    private double lastPosition;
    private double perimeter = 4 * Math.PI;
    private double ticksPerRevolution = 1120 * 25;
    private double inchesPerTick = perimeter / ticksPerRevolution;
    private double ticksPerInch = 1 / inchesPerTick;
    private double ticksPerSpin = ticksPerRevolution * 40;
    private double diameter = 18;
    private double setPoint;
    private double startPosition;
    private double incPerTile = 24;
    private double integral = 0;
    private double derivative = 0;


    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction, rotation;
    boolean aButton, bButton, touched;
    public PIDController pidRotate, pidDrive;


    private float skyStoneX = 0;
    private float Stone1X = 0;
    private float Stone2X = 0;
    int h = 0;
    int skystonePostion;
    int Stone1Postion;
    int Stone2Postion;
    int seeSkystone = 0;
    int seeStone1 = 0;
    int seeStone2 = 0;
    boolean canSeeSkystone = false;


    private static final String VUFORIA_KEY =
            "AVHZDTL/////AAABmQcZurBiA01smn3EpdcPCJpZqB8HZL60ujXKBU3ejemhikdsno1L3+7QKhYWSXEfUl5uWZxBqPJXl6Qj0AG3XKuq/jLKmyLJ67xHlYM/LoVKbxhjxGJJ5stO+21qtYET0KberI6XObNkTmskQ8kLQX7QwLhmllfyhu25bPFWwmVdnGq3jRAxoCNKP9ktqKkqp62Fl39qcvOwCOBPqG0uFMFHwVaNavRHS1f4fnuZXk4QqEDo5e2K9J/sCR/2BvvzdPV3QfTkUPNm/8dfW2nsxCM2E9rpj67CFq9fOAHjY+7tp4o2U/yJbxc5RBr5mZ9/CeQk7zfl9rQv7WrVWevfvHqvb2xMsoqVJGze9rE62AmI";


    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;


    public void timer(long miliseconds) {
        long x = (long) elapsedTime.milliseconds();
        while (x < miliseconds + (long) elapsedTime.milliseconds() && opModeIsActive()) {
        }
    }

    public void driveInches(double inches) {

        startPosition = Math.abs(ldrive1.getCurrentPosition()) / ticksPerInch;
        lastError = 0;
        errorT = inches;
        if (inches > 0) {
            while (errorT > 0 && opModeIsActive()) {
                currentPosition = Math.abs(ldrive1.getCurrentPosition()) / ticksPerInch - startPosition;
                errorT = inches - currentPosition;

                integral += errorT;
                if (errorT <= 0) {
                    integral = 0;
                }
                if (errorT > 24) {
                    integral = 0;
                }
                derivative = errorT - lastError;
                lastError = errorT;

                uT = errorT * kp + integral * ki + derivative * kd;

                ldrive1.setPower(uT);
                ldrive2.setPower(uT);
                rdrive1.setPower(uT);
                rdrive2.setPower(uT);

                telemetry.addData("uT", uT);
                telemetry.addData("errorT", errorT);
                telemetry.addData("start", startPosition);

                sleep(10);
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
            telemetry.addData("rdrive:", rdrive1.getPower());
            telemetry.addData("ldrive:", ldrive1.getPower());
            telemetry.update();
        } else {
            while (errorT > 0 && opModeIsActive()) {
                currentPosition = Math.abs(ldrive1.getCurrentPosition()) / ticksPerInch - startPosition;
                errorT = -inches - currentPosition;

                integral += errorT;
                if (errorT <= 0) {
                    integral = 0;
                }
                if (errorT > 24) {
                    integral = 0;
                }
                derivative = errorT - lastError;
                lastError = errorT;

                uT = errorT * kp + integral * ki + derivative * kd;

                ldrive1.setPower(uT);
                ldrive2.setPower(uT);
                rdrive1.setPower(uT);
                rdrive2.setPower(uT);

                sleep(10);

            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
        }
    }

    // Turns right by default
    public void turnDeg(double deg) {
        startPosition = ldrive1.getCurrentPosition() / ticksPerInch;
        lastError = 0;
        if (deg > 0) {
            while (errorT > 0 && opModeIsActive()) {
                currentPosition = Math.abs(ldrive1.getCurrentPosition()) / ticksPerInch - startPosition;
                errorT = (diameter * Math.PI) * (deg / 360) - currentPosition;

                integral += errorT;
                if (errorT <= 0) {
                    integral = 0;
                }
                if (errorT > 24) {
                    integral = 0;
                }
                derivative = errorT - lastError;
                lastError = errorT;

                uT = errorT * kp + integral * ki + derivative * kd;

                ldrive1.setPower(uT);
                ldrive2.setPower(uT);
                rdrive1.setPower(-uT);
                rdrive2.setPower(-uT);

                sleep(10);
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
            slide1.setPower(0);
            slide2.setPower(0);
        } else {
            while (errorT > 0 && opModeIsActive()) {
                currentPosition = Math.abs(rdrive1.getCurrentPosition()) / ticksPerInch - startPosition;
                errorT = (diameter * Math.PI) * (deg / 360) - currentPosition;

                integral += errorT;
                if (errorT <= 0) {
                    integral = 0;
                }
                if (errorT > 24) {
                    integral = 0;
                }
                derivative = errorT - lastError;
                lastError = errorT;

                uT = errorT * kp + integral * ki + derivative * kd;

                ldrive1.setPower(-uT);
                ldrive2.setPower(-uT);
                rdrive1.setPower(uT);
                rdrive2.setPower(uT);

                sleep(10);
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
            slide1.setPower(0);
            slide2.setPower(0);
        }
    }

    public void slideInches(double inches) {

        startPosition = slide1.getCurrentPosition() / ticksPerInch;
        lastError = 0;
        if (inches > 0) {
            while (errorT > 0 && opModeIsActive()) {
                currentPosition = Math.abs(slide1.getCurrentPosition()) / ticksPerInch - startPosition;
                errorT = inches - currentPosition;

                integral += errorT;
                if (errorT <= 0) {
                    integral = 0;
                }
                if (errorT > 24) {
                    integral = 0;
                }
                derivative = errorT - lastError;
                lastError = errorT;

                uT = errorT * kp + integral * ki + derivative * kd;

                slide1.setPower(uT);
                slide2.setPower(uT);
                sleep(10);
            }
            slide1.setPower(0);
            slide2.setPower(0);
        } else {
            while (errorT > 0 && opModeIsActive() && uT > 0) {
                currentPosition = Math.abs(slide1.getCurrentPosition()) / ticksPerInch - startPosition;
                errorT = -inches - currentPosition;

                integral += errorT;
                if (errorT <= 0) {
                    integral = 0;
                }
                if (errorT > 24) {
                    integral = 0;
                }
                derivative = errorT - lastError;
                lastError = errorT;

                uT = errorT * kp + integral * ki + derivative * kd;

                slide1.setPower(uT);
                slide2.setPower(uT);
                sleep(10);
            }
            slide1.setPower(0);
            slide2.setPower(0);
        }
    }

    private void Elevator(double inches){
        startPosition = elevator.getCurrentPosition();
        if (inches > 0) {
            while (errorT > 0 && opModeIsActive()) {
                errorT = inches - elevator.getCurrentPosition() / ticksPerInch - startPosition;
                uT = errorT * kp;
                elevator.setPower(uT);
            }
        } else {
            while (errorT > 0 && opModeIsActive()) {
                errorT = -inches - elevator.getCurrentPosition() / ticksPerInch - startPosition;
                uT = errorT * kp;
                elevator.setPower(-uT);
            }
        }
    }

        public void drivaBySpin(double spins){
            setPoint = ldrive1.getCurrentPosition() / ticksPerSpin;

            if (spins > 0) {
                errorT = (spins + setPoint) / 2;
                lastPosition = 0;
                while (errorT > setPoint && opModeIsActive() && uT > 0.3) {

                    uT = kp * errorT;
                    currentPosition = ldrive1.getCurrentPosition() / ticksPerSpin;
                    errorT -= currentPosition - lastPosition;
                    lastPosition = currentPosition;

                    ldrive1.setPower(uT);
                    ldrive2.setPower(uT);
                    rdrive1.setPower(uT);
                    rdrive2.setPower(uT);
                }
                ldrive1.setPower(0);
                ldrive2.setPower(0);
                rdrive1.setPower(0);
                rdrive2.setPower(0);
            } else {
                errorT = -spins + setPoint;
                lastPosition = 0;
                if (errorT > setPoint && opModeIsActive() && uT > 0.3) {
                    uT = kp * errorT;

                    currentPosition = Math.abs(ldrive1.getCurrentPosition()) / ticksPerInch;
                    errorT -= Math.abs(currentPosition - lastPosition);
                    lastPosition = currentPosition;

                    ldrive1.setPower(-uT);
                    ldrive2.setPower(-uT);
                    rdrive1.setPower(-uT);
                    rdrive2.setPower(-uT);
                }
                ldrive1.setPower(0);
                ldrive2.setPower(0);
                rdrive1.setPower(0);
                rdrive2.setPower(0);
            }
        }

    /*private void collect(){
        while(!cubeIn.isPressed()){
            collectLeft.setPosition(0.2);
            collectRight.setPosition(0.2);
            ldrive1.setPower(0.3);
            ldrive2.setPower(0.3);
            rdrive1.setPower(0.3);
            rdrive2.setPower(0.3);
        }
    }

    private void deploy(){
        ldrive1.setPower(-0.3);
        ldrive2.setPower(-0.3);
        rdrive1.setPower(-0.4);
        rdrive2.setPower(-0.4);
        //collectLeft.setPosition(0.7);
        //collectRight.setPosition(0.7);
        sleep(500);
        ldrive1.setPower(0);
        ldrive2.setPower(0);
        rdrive1.setPower(0);
        rdrive2.setPower(0);
        collectLeft.setPosition(0);
        collectRight.setPosition(0);
    }*/

        // For the closest placement of the skystone
        private void caseSSP () {
        /*switch (skystonePostion) {
            case 1:
                driveInches(-14);
            case 2:
                driveInches(-22);
            case 3:
                driveInches(-30);
        }*/
            sleep(300);
            slideInches(31.75);
            sleep(300);
            double x = ldrive1.getCurrentPosition() / ticksPerInch;
            //collect();
            double y = ldrive1.getCurrentPosition() / ticksPerInch - x;
            sleep(300);
            driveInches(-y);
            sleep(300);
            slideInches(-8.75);
            sleep(300);
            turnDeg(180);
            sleep(300);
            switch (skystonePostion) {
                case 1:
                    driveInches(88);
                case 2:
                    driveInches(80);
                case 3:
                    driveInches(72);
            }
            sleep(300);
            //deploy();
            sleep(300);
            driveInches(-48);
        }

        @Override
        public void runOpMode () throws InterruptedException {
            rdrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
            rdrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
            ldrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
            ldrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
            slide1 = hardwareMap.get(DcMotor.class, "slide1");
            slide2 = hardwareMap.get(DcMotor.class, "slide2");
            elevator = hardwareMap.get(DcMotor.class, "elevator");
            hold = hardwareMap.get(CRServo.class, "hold");
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


            pidRotate = new PIDController(.003, .00003, 0);


            pidDrive = new PIDController(.05, 0, 0);


            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);


            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);
                idle();
            }

            waitForStart();
            rdrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            rdrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            ldrive1.setDirection(DcMotorSimple.Direction.FORWARD);
            ldrive2.setDirection(DcMotorSimple.Direction.FORWARD);
            slide1.setDirection(DcMotorSimple.Direction.FORWARD);
            slide2.setDirection(DcMotorSimple.Direction.FORWARD);
            elevator.setDirection(DcMotorSimple.Direction.FORWARD);
            hold.setDirection(DcMotorSimple.Direction.FORWARD);
        /*collectRight.setDirection(Servo.Direction.REVERSE);
        collectLeft.setDirection(Servo.Direction.FORWARD);*/

            //paramaters for stright drive


            while (opModeIsActive() && h == 0) {

                driveInches(48);

                h++;
                telemetry.addData("h:", h);
                telemetry.update();

                //Elevator(-4);
                //telemetry.addData("", 4);
                //telemetry.update();
                //sleep(150);
                //turnDeg(90);
            /*sleep(500);
            slideInches(32.5);
            sleep(1250);
            driveInches(-48);
            sleep(2000);*/
            }
        }

        private void initVuforia () {

            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


            vuforia = ClassFactory.getInstance().createVuforia(parameters);


        }


        //init the TensorFlow
        private void initTfod () {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.5;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        }

        //if see 3 diffrent objects
        private int seeThreeObj (List < Recognition > Recognitions) {

            if (Recognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT)) {
                skyStoneX = Recognitions.get(0).getLeft();
            } else if (Recognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT)) {
                Stone1X = Recognitions.get(0).getLeft();
            }


            if (Recognitions.get(1).getLabel().equals(LABEL_SECOND_ELEMENT)) {
                skyStoneX = Recognitions.get(1).getLeft();

            } else if (Recognitions.get(1).getLabel().equals(LABEL_FIRST_ELEMENT)) {

                if (Stone1X != 0) {

                    Stone2X = Recognitions.get(1).getLeft();
                } else if (Stone1X == 0) {
                    Stone1X = Recognitions.get(1).getLeft();
                }
            }

            if (Recognitions.get(2).getLabel().equals(LABEL_SECOND_ELEMENT)) {
                skyStoneX = Recognitions.get(2).getLeft();
            } else if (Recognitions.get(2).getLabel().equals(LABEL_FIRST_ELEMENT)) {
                Stone2X = Recognitions.get(2).getLeft();
            }


            if (skyStoneX < Stone1X && skyStoneX < Stone2X) {
                skystonePostion = 1;
            } else if (skyStoneX > Stone1X && skyStoneX > Stone2X) {
                skystonePostion = 3;
            } else if (skyStoneX > Stone1X && skyStoneX < Stone2X || skyStoneX < Stone1X && skyStoneX > Stone2X) {
                skystonePostion = 2;
            }

            return skystonePostion;
        }


        private void resetAngle () {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            globalAngle = 0;
        }

        /**
         * Get current cumulative angle rotation from last reset.
         *
         * @return Angle in degrees. + = left, - = right from zero point.
         */
        private double getAngle () {
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
        public void rotate ( int degrees, double power){
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
                    power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                    ldrive1.setPower(-power);
                    ldrive2.setPower(-power);
                    rdrive1.setPower(power);
                    rdrive2.setPower(power);
                } while (opModeIsActive() && !pidRotate.onTarget());
            } else    // left turn.
                do {
                    power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                    ldrive1.setPower(-power);
                    ldrive2.setPower(-power);
                    rdrive1.setPower(power);
                    rdrive2.setPower(power);
                } while (opModeIsActive() && !pidRotate.onTarget());

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


    }




