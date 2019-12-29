
package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.android.dex.EncodedValueReader;

import java.util.List;

@Autonomous(name = "Auto 11226", group = "Autonomous")
public class Auto_11226_B extends LinearOpMode {
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
    private Servo collectRight = null;
    private Servo collectLeft = null;
    private TouchSensor cubeIn = null;

    private double kp = 0.019;
    private double ks = 0.1;
    private double uT = 1;
    private double errorT;
    private double currentPosition;
    private double lastPosition;
    private double perimeter = 4 * Math.PI;
    private double ticksPerRevolution = 1120;
    private double inchesPerTick = perimeter / ticksPerRevolution;
    private double ticksPerSpin = ticksPerRevolution * 40;
    private double ticksPerInch = 1 / inchesPerTick;
    private double diameter = 18;
    private double setPoint;
    private double incPerTile = 24;


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


    private static final String VUFORIA_KEY =
            "AVHZDTL/////AAABmQcZurBiA01smn3EpdcPCJpZqB8HZL60ujXKBU3ejemhikdsno1L3+7QKhYWSXEfUl5uWZxBqPJXl6Qj0AG3XKuq/jLKmyLJ67xHlYM/LoVKbxhjxGJJ5stO+21qtYET0KberI6XObNkTmskQ8kLQX7QwLhmllfyhu25bPFWwmVdnGq3jRAxoCNKP9ktqKkqp62Fl39qcvOwCOBPqG0uFMFHwVaNavRHS1f4fnuZXk4QqEDo5e2K9J/sCR/2BvvzdPV3QfTkUPNm/8dfW2nsxCM2E9rpj67CFq9fOAHjY+7tp4o2U/yJbxc5RBr5mZ9/CeQk7zfl9rQv7WrVWevfvHqvb2xMsoqVJGze9rE62AmI";


    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;


    public void timer(long miliseconds) {
        long x = (long) elapsedTime.milliseconds();
        while (x < miliseconds + (long) elapsedTime.milliseconds() && opModeIsActive()) {}
    }

    public void driveInches(double inches) {

        setPoint = ldrive1.getCurrentPosition() / ticksPerInch;
        lastPosition = setPoint;

        if (inches > 0) {
            errorT = inches + setPoint;
            int x = 0;
            while(x == 0) {
                while(errorT > setPoint && opModeIsActive()) {
                    uT = kp * errorT;
                    currentPosition = ldrive1.getCurrentPosition() / ticksPerInch;
                    errorT -= currentPosition - lastPosition;
                    lastPosition = currentPosition;

                    ldrive1.setPower(uT);
                    ldrive2.setPower(uT);
                    rdrive1.setPower(uT);
                    rdrive2.setPower(uT);
                    telemetry.addData("start", setPoint);
                    telemetry.addData("error", errorT);
                    telemetry.update();
                }
                ldrive1.setPower(0);
                ldrive2.setPower(0);
                rdrive1.setPower(0);
                rdrive2.setPower(0);
                x++;
            }
            telemetry.addData("rdrive:", rdrive1.getPower());
            telemetry.addData("ldrive:", ldrive1.getPower());
            telemetry.update();
        } else {
            errorT = -inches + setPoint;
            int x = 0;
            while(x == 0) {
                if (errorT > setPoint && opModeIsActive()) {
                    uT = kp * errorT;

                    currentPosition = Math.abs(ldrive1.getCurrentPosition()) / ticksPerInch;
                    errorT -= Math.abs(currentPosition - lastPosition);
                    lastPosition = currentPosition;

                    ldrive1.setPower(-uT);
                    ldrive2.setPower(-uT);
                    rdrive1.setPower(-uT);
                    rdrive2.setPower(-uT);
                }
                else{
                    x++;
                }
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
        }
    }

    // Turns right by default
    public void turnDeg(double deg) {
        setPoint = ldrive1.getCurrentPosition() / ticksPerInch;
        lastPosition = 0;
        if (deg > 0) {
            errorT = diameter * Math.PI * (deg / 360);
            int x = 0;
            while(x == 0) {
                if (errorT > 0 && opModeIsActive()) {

                    currentPosition = ldrive1.getCurrentPosition() / ticksPerInch - setPoint;
                    errorT -= (currentPosition - lastPosition);
                    lastPosition = currentPosition;

                    uT = kp * errorT;

                    ldrive1.setPower(uT);
                    ldrive2.setPower(uT);
                    rdrive1.setPower(-uT);
                    rdrive2.setPower(-uT);
                    telemetry.addData("", uT);
                    telemetry.update();
                }
                else{
                    x++;
                }
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
            slide1.setPower(0);
            slide2.setPower(0);
        } else {
            errorT = diameter * Math.PI * (-deg / 360) + setPoint;
            int x = 0;
            while(x == 0) {
                if (errorT > setPoint && opModeIsActive()) {
                    uT = kp * errorT;

                    currentPosition = rdrive1.getCurrentPosition() / ticksPerInch;
                    errorT -= currentPosition - lastPosition;
                    lastPosition = currentPosition;

                    ldrive1.setPower(-uT);
                    ldrive2.setPower(-uT);
                    rdrive1.setPower(uT);
                    rdrive2.setPower(uT);
                    telemetry.addData("rdrive:", rdrive1.getPower());
                    telemetry.addData("ldrive:", ldrive1.getPower());
                    telemetry.update();
                }
                else{
                    x++;
                }
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

        setPoint = slide1.getCurrentPosition() / ticksPerInch;
        lastPosition = setPoint;

        if (inches > 0) {
            errorT = inches;
            int x = 0;
            while (x == 0) {
                if (errorT > 0 && opModeIsActive()) {
                    uT = ks * errorT;

                    currentPosition = slide1.getCurrentPosition() / ticksPerInch - setPoint;
                    errorT -= (currentPosition - lastPosition);
                    lastPosition = currentPosition;

                    slide1.setPower(-uT);
                    slide2.setPower(-uT);

                    telemetry.addData("errorT", errorT);
                    telemetry.addData("uT", uT);
                    telemetry.update();
                }
                else{
                    x++;
                }
            }
            slide1.setPower(0);
            slide2.setPower(0);
        } else {
            errorT = -inches;
            int x = 0;
            while(x == 0) {
                if (errorT > 0 && opModeIsActive()) {
                    uT = ks * errorT;

                    currentPosition = Math.abs(slide1.getCurrentPosition()) / ticksPerInch - setPoint;
                    errorT -= Math.abs(currentPosition - lastPosition);
                    lastPosition = currentPosition;

                    slide1.setPower(uT);
                    slide2.setPower(uT);
                }
                else{
                    x++;
                }
            }
            slide1.setPower(0);
            slide2.setPower(0);
        }
    }

    private void Elevator(double spins) {
        telemetry.addData("", ks);
        telemetry.update();
        setPoint = elevator.getCurrentPosition();
        if (spins > 0) {
            while (elevator.getCurrentPosition() < ticksPerRevolution * spins && opModeIsActive()) {
                elevator.setPower(-1);
            }
            elevator.setPower(0);
            sleep(500);
        } else {
            while (elevator.getCurrentPosition() > ticksPerRevolution * spins && opModeIsActive()) {
                elevator.setPower(1);
            }
            elevator.setPower(0);
        }
        sleep(500);
    }

    public void drivaBySpin(double spins){
        setPoint = ldrive1.getCurrentPosition() / ticksPerSpin;

        if (spins > 0) {
            errorT = (spins + setPoint) / 2;
            lastPosition = 0;
            int x = 0;
            while(x == 0) {
                while(errorT > setPoint && opModeIsActive() && uT > 0.3) {

                    uT = kp * errorT;
                    currentPosition = ldrive1.getCurrentPosition() / ticksPerSpin;
                    errorT -= currentPosition - lastPosition;
                    lastPosition = currentPosition;

                    ldrive1.setPower(uT);
                    ldrive2.setPower(uT);
                    rdrive1.setPower(uT);
                    rdrive2.setPower(uT);
                }
                x++;
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
        } else {
            errorT = -spins + setPoint;
            lastPosition = 0;
            int x = 0;
            while(x == 0) {
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
                else{
                    x++;
                }
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
        }
    }

    private void collect(){
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
        collectLeft.setPosition(0.7);
        collectRight.setPosition(0.7);
        sleep(500);
        ldrive1.setPower(0);
        ldrive2.setPower(0);
        rdrive1.setPower(0);
        rdrive2.setPower(0);
        collectLeft.setPosition(0);
        collectRight.setPosition(0);
    }

    // For the closest placement of the skystone
    private void caseSSP(){
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
        collect();
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
        deploy();
        sleep(300);
        driveInches(-48);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        rdrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        ldrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        collectRight = hardwareMap.get(Servo.class, "collect right");
        collectLeft = hardwareMap.get(Servo.class, "collect left");
        cubeIn = hardwareMap.get(TouchSensor.class, "cube in");

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

        waitForStart();
        rdrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rdrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        ldrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        ldrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide1.setDirection(DcMotorSimple.Direction.FORWARD);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        collectRight.setDirection(Servo.Direction.REVERSE);
        collectLeft.setDirection(Servo.Direction.FORWARD);

        int h = 0;
        while (opModeIsActive()) {

            //while(ldrive1.isBusy()){}
            //Elevator(10);
            while(h == 0) {
                slideInches(48);
                h++;
            }
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
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    //if see 3 diffrent objects
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
}




