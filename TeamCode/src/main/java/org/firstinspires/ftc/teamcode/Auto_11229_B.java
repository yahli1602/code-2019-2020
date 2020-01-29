
package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.internal.android.dex.EncodedValueReader;
@Disabled
@Autonomous(name = "Auto 11229", group = "Autonomous")
public class Auto_11229_B extends LinearOpMode {

    ElapsedTime elapsedTime = new ElapsedTime();
    public DcMotor rdrive1 = null;
    public DcMotor rdrive2 = null;
    public DcMotor ldrive1 = null;
    public DcMotor ldrive2 = null;
    public DcMotor slide = null;
    public DcMotor elevator = null;
    //private Servo collectRight = null;
    //private Servo collectLeft = null;
    // public TouchSensor stoneIn = null;

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
    private double diameter = 13.979921;
    private double setPoint;
    private double lim;
    private double magnif;

    public void timer(long miliseconds) {
        long x = (long) elapsedTime.milliseconds();
        while (x < miliseconds + (long) elapsedTime.milliseconds() && opModeIsActive()) {}
    }

    public void driveInches(double inches) {

        setPoint = ldrive1.getCurrentPosition() / ticksPerInch;
        lastPosition = setPoint;

        if (inches > 0) {
            errorT = inches;
            lim = Math.sqrt(0.005967 * inches - 0.053208);
            if(lim < 0.01 * lim + 0.03){
                lim = 0.14;
            }
            magnif = Math.sqrt(-6.75 * lim + 3.525);
            if(magnif > 1.5){
                magnif = 1.5;
            }
            int x = 0;
            while(x == 0) {
                while(errorT > 0 && opModeIsActive() && uT > lim) {
                    uT = kp * errorT;
                    currentPosition = ldrive1.getCurrentPosition() / ticksPerInch - setPoint;
                    errorT -= currentPosition - lastPosition;
                    lastPosition = currentPosition;

                    ldrive1.setPower(uT);
                    ldrive2.setPower(uT);
                    rdrive1.setPower(magnif * uT);
                    rdrive2.setPower(magnif * uT);
                    telemetry.addData("start", setPoint);
                    telemetry.addData("error", errorT);
                    telemetry.addData("limit", lim);
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
            errorT = -inches;
            int x = 0;
            while(x == 0) {
                if (errorT > 0 && opModeIsActive() && uT > lim) {
                    uT = kp * errorT;

                    currentPosition = Math.abs(ldrive1.getCurrentPosition()) / ticksPerInch - setPoint;
                    errorT -= Math.abs(currentPosition - lastPosition);
                    lastPosition = currentPosition;

                    ldrive1.setPower(-uT);
                    ldrive2.setPower(-uT);
                    rdrive1.setPower(-magnif * uT);
                    rdrive2.setPower(-magnif * uT);
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
        lim = Math.sqrt(0.00044434169 * deg - 0.0259485021);
        if (deg > 0) {
            errorT = diameter * Math.PI * (deg / 360);
            int x = 0;
            while(x == 0) {
                if (errorT > 0 && opModeIsActive() && uT > lim) {

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
            slide.setPower(0);
        } else {
            errorT = diameter * Math.PI * (-deg / 360);
            int x = 0;
            while(x == 0) {
                if (errorT > 0 && opModeIsActive() && uT > lim) {
                    uT = kp * errorT;

                    currentPosition = rdrive1.getCurrentPosition() / ticksPerInch - setPoint;
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
            slide.setPower(0);
        }
    }

    public void slideInches(double inches) {

        setPoint = slide.getCurrentPosition() / ticksPerInch;
        lastPosition = setPoint;

        if (inches > 0) {
            errorT = inches + setPoint;
            int x = 0;
            while (x == 0) {
                if (errorT > 0 && opModeIsActive() && uT > 0.3) {
                    uT = ks * errorT;

                    currentPosition = slide.getCurrentPosition() / ticksPerInch - setPoint;
                    errorT -= (currentPosition - lastPosition);
                    lastPosition = currentPosition;

                    slide.setPower(-uT);
                }
                else{
                    x++;
                }
            }
            slide.setPower(0);
        } else {
            errorT = -inches;
            int x = 0;
            while(x == 0) {
                if (errorT > 0 && opModeIsActive() && uT > 0.3) {
                    uT = ks * errorT;

                    currentPosition = Math.abs(slide.getCurrentPosition()) / ticksPerInch - setPoint;
                    errorT -= Math.abs(currentPosition - lastPosition);
                    lastPosition = currentPosition;

                    slide.setPower(uT);
                }
                else{
                    x++;
                }
            }
            slide.setPower(0);
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
            errorT = spins / 2;
            lastPosition = 0;
            int x = 0;
            while(x == 0) {
                while(errorT > 0 && opModeIsActive() && uT > 0.3) {

                    uT = kp * errorT;
                    currentPosition = ldrive1.getCurrentPosition() / ticksPerSpin - setPoint;
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
            errorT = -spins;
            lastPosition = 0;
            int x = 0;
            while(x == 0) {
                if (errorT > 0 && opModeIsActive() && uT > 0.3) {
                    uT = kp * errorT;

                    currentPosition = Math.abs(ldrive1.getCurrentPosition()) / ticksPerInch - setPoint;
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

    /*private void collect(){
        while(!touch.isPressed()){
            collectLeft.setPosition(0.2);
            collectRight.setPosition(0.2);
        }
        collectRight.setPosition(0);
        collectRight.setPosition(0);
    }

    private void spit(){
        while(touch.isPressed()){
            collectLeft.setPosition(0.7);
            collectRight.setPosition(0.7);
        }
        sleep(500);
        collectRight.setPosition(0);
        collectRight.setPosition(0);
    }*/

    private void case1(){
        driveInches(11);
        sleep(300);
        turnDeg(-20);
        sleep(200);
        //collect()
        driveInches(9);
        sleep(300);
        driveInches(-9);
        sleep(300);
        turnDeg(110);
        sleep(300);
    }

    private void case2(){
        driveInches(14);
        sleep(300);
        //collect()
        turnDeg(-45);
        sleep(300);
        turnDeg(135);
        sleep(300);
    }

    private void case3(){
        driveInches(14);
        sleep(300);
        //collect()
        turnDeg(45);
        sleep(300);
        turnDeg(45);
        sleep(300);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        rdrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        ldrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        //collectLeft = hardwareMap.get(Servo.class, "collect left");
        //collectRight = hardwareMap.get(Servo.class, "collect right");
        //stoneIn = hardwareMap.get(TouchSensor.class, "cubeIn");
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        rdrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rdrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rdrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        rdrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rdrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        ldrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        ldrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        //collectRight.setDirection(Servo.Direction.REVERSE);
        //collectLeft.setDirection(Servo.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);

        int h = 0;
        while (opModeIsActive()) {

            //while(ldrive1.isBusy()){}
            //Elevator(10);
            while(h == 0) {
                driveInches(11);
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
}




