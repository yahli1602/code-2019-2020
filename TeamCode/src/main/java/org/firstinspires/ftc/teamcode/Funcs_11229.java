package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Funcs 11229", group = "Linear Opmode")
public class Funcs_11229 extends LinearOpMode {

    private ElapsedTime elapsedTime = new ElapsedTime();
    //driving motors
    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    //elevator
    private DcMotor elevator = null;


    //hardware map
    public void init(HardwareMap HM) {

        rDrive1 = HM.get(DcMotor.class, "rDrive1");
        rDrive2 = HM.get(DcMotor.class, "rDrive2");
        lDrive1 = HM.get(DcMotor.class, "lDrive1");
        lDrive2 = HM.get(DcMotor.class, "lDrive2");
        slide = HM.get(DcMotor.class, "slide");
        elevator = HM.get(DcMotor.class, "elevator");

//set Direction

        rDrive1.setDirection(DcMotor.Direction.FORWARD);
        rDrive2.setDirection(DcMotor.Direction.FORWARD);
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);
    }
//Functions for Teleop

    //timer
    public void timer(long miliseconds) {
        long x = (long) elapsedTime.milliseconds();
        while (x < miliseconds + (long) elapsedTime.milliseconds()) {
        }
    }

    //Drive
    public void drive() {
        //Drive/turn
        if (gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2) {
            rDrive1.setPower(gamepad1.right_stick_y);
            rDrive2.setPower(gamepad1.right_stick_y);
        } else if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
            lDrive1.setPower(gamepad1.left_stick_y);
            lDrive2.setPower(gamepad1.left_stick_y);
        } else {
            rDrive1.setPower(0);
            rDrive2.setPower(0);
            lDrive1.setPower(0);
            lDrive2.setPower(0);

        }
        //slide
        if (gamepad1.right_trigger > 0) {
            slide.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0) {
            slide.setPower(gamepad1.left_trigger);
        } else {
            slide.setPower(0);
        }
    }


    //elevator
    public void elevator() {
        if (gamepad2.right_stick_y > 0.2) {
            elevator.setPower(gamepad2.right_stick_y);
        } else if (gamepad2.right_stick_y < 0.2) {
            elevator.setPower(gamepad2.right_stick_y);
        } else {
            elevator.setPower(0);
        }
    }

    public void collect() {

    }

    //grabbing the build plate
    public void grabber() {

    }


//Functions for Autonomous


    public void rotateByDirction(String direction, float rotationPower) { //rotation by direction call it again to turn off
        if (rDrive1.isBusy()) {
            lDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive1.setPower(0);
            rDrive2.setPower(0);
        } else {
            if (direction == "right") {
                while (rotationPower > 0) {
                    lDrive1.setPower(rotationPower);
                    lDrive2.setPower(rotationPower);
                    rDrive1.setPower(-rotationPower);
                    rDrive2.setPower(-rotationPower);
                }
            } else if (direction == "left") {
                while (rotationPower > 0) {
                    lDrive1.setPower(-rotationPower);
                    lDrive2.setPower(-rotationPower);
                    rDrive1.setPower(rotationPower);
                    rDrive2.setPower(rotationPower);
                }
            }
        }


    }


    public void spinUntilTarget(vuforiaSkystone11229 target) {

    }


    @Override
    public void runOpMode() {


    }
}