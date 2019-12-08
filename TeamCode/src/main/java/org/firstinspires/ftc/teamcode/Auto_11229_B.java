
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dex.EncodedValueReader;

@Autonomous(name = "Auto 11229", group = "Autonomous")
public class Auto_11229_B extends LinearOpMode {

    ElapsedTime elapsedTime = new ElapsedTime();
    public DcMotor rdrive1 = null;
    public DcMotor rdrive2 = null;
    public DcMotor ldrive1 = null;
    public DcMotor ldrive2 = null;
    public DcMotor slide = null;
    public DcMotor elevator = null;

    private double kp = 0.05;
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
    private double VerticalLength = 15.5;
    private double setPoint;
    private double target;

    public void timer(long miliseconds) {
        long x = (long) elapsedTime.milliseconds();
        while (x < miliseconds + (long) elapsedTime.milliseconds() && opModeIsActive()) {
        }
    }

    public void driveInches(double inches) {

        setPoint = ldrive1.getCurrentPosition();
        /*ldrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rdrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        if(inches > 0){
            errorT = inches;
            lastPosition = 0;
            while (errorT > setPoint && opModeIsActive() && uT > 0.3) {
                uT = kp * errorT;

                currentPosition = ldrive1.getCurrentPosition() / ticksPerInch;
                errorT -= currentPosition - lastPosition;
                lastPosition = currentPosition;

                ldrive1.setPower(uT);
                ldrive2.setPower(uT);
                rdrive1.setPower(1.55 * uT);
                rdrive2.setPower(1.55 * uT);
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
        }
        else{
            errorT = -inches;
            lastPosition = 0;
            while (errorT > setPoint && opModeIsActive() && uT > 0.3) {
                uT = kp * errorT;

                currentPosition = -ldrive1.getCurrentPosition() / ticksPerInch;
                errorT -= (currentPosition - lastPosition);
                lastPosition = currentPosition;

                ldrive1.setPower(-uT);
                ldrive2.setPower(-uT);
                rdrive1.setPower(-uT * 1.55);
                rdrive2.setPower(-uT * 1.55);
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
        }
    }
    // Turns right by default
    public void turnDeg(double deg){

        setPoint = ldrive1.getCurrentPosition();
        /*ldrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rdrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        if(deg > 0){
            errorT = diameter * Math.PI * (deg/360);
            lastPosition = 0;
            while (errorT > setPoint && opModeIsActive() && uT > 0.3) {
                uT = kp * errorT;

                currentPosition = ldrive1.getCurrentPosition() / ticksPerInch;
                errorT -= (currentPosition - lastPosition);
                lastPosition = currentPosition;

                ldrive1.setPower(uT);
                ldrive2.setPower(uT);
                rdrive1.setPower(-uT);
                rdrive2.setPower(-uT);
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
        }
        else{
            errorT = diameter * Math.PI * (-deg/360);
            lastPosition = 0;
            while (errorT > setPoint && opModeIsActive() && uT > 0.3) {
                uT = kp * errorT;

                currentPosition = -ldrive1.getCurrentPosition() / ticksPerInch;
                errorT -= (currentPosition - lastPosition);
                lastPosition = currentPosition;

                ldrive1.setPower(-uT);
                ldrive2.setPower(-uT);
                rdrive1.setPower(uT);
                rdrive2.setPower(uT);

                while(ldrive1.isBusy()){}
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
        }
    }

    public void slideInches(double inches){

        setPoint = slide.getCurrentPosition();
        /*slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        if(inches > 0){
            errorT = inches;
            lastPosition = 0;
            while (errorT > setPoint && opModeIsActive() && uT > 0.3) {
                uT = ks * errorT;

                currentPosition = slide.getCurrentPosition() / ticksPerInch;
                errorT -= (currentPosition - lastPosition);
                lastPosition = currentPosition;

                slide.setPower(-uT);
            }
            slide.setPower(0);
        }
        else{
            errorT = -inches;
            lastPosition = 0;
            while (errorT > setPoint && opModeIsActive() && uT > 0.3) {
                uT = ks * errorT;

                currentPosition = -slide.getCurrentPosition() / ticksPerInch;
                errorT -= (currentPosition - lastPosition);
                lastPosition = currentPosition;

                slide.setPower(uT);
            }
            slide.setPower(0);
        }
    }

    private void holdElevator(){
        while(elevator.getCurrentPosition() < ticksPerRevolution * 12 && opModeIsActive()){
            elevator.setPower(-1);
        }
        elevator.setPower(0);
        sleep(500);
        while(elevator.getCurrentPosition() > 0 && opModeIsActive()){
            elevator.setPower(1);
        }
        elevator.setPower(0);
    }

    public void AutoSideRight(){
        //holdElevator();
        driveInches(4);
        turnDeg(-90);
        driveInches(72);
        turnDeg(90);
        driveInches(24);
        //Vuforia stuff
        /*driveInches(12.5);
        // case 1
        turnDeg(130.5);
        driveInches(36.88);
        turnDeg(-40.5);
        // case 2
        turnDeg(126.7);
        driveInches(43.27);
        turnDeg(-36.7);
        // case 3
        turnDeg(118.61);
        driveInches(50.12);
        turnDeg(-28.61);
        driveInches(24);
        driveInches(-24);*/
    }

    @Override
    public void runOpMode() throws InterruptedException {
        rdrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        ldrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "elevator");

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
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);

        while (opModeIsActive()) {
            turnDeg(-90);
            //while(ldrive1.isBusy()){}
            holdElevator();
            sleep(500);
            driveInches(72);
            /*while(ldrive1.isBusy()){}
            turnDeg(90);
            while(ldrive1.isBusy()){}
            driveInches(24);*/
            /*target = diameter * (90/360) * ticksPerInch;
            ldrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ldrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rdrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rdrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rdrive1.setTargetPosition((int)target);
            rdrive2.setTargetPosition((int)target);
            ldrive1.setTargetPosition(-(int)target);
            ldrive2.setTargetPosition(-(int)target);
            ldrive1.setPower(-0.7);
            ldrive2.setPower(-0.7);
            rdrive1.setPower(0.7);
            rdrive1.setPower(0.7);*/

        }
    }
}




