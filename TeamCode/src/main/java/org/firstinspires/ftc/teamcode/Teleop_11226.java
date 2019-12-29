
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Teleop 11226", group = "Linear Opmode")
public class Teleop_11226 extends LinearOpMode {
    private boolean pinch = false;
    private boolean pushCube = false;
    //driving motors
    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide1 = null;
    private DcMotor slide2 = null;
    //elevator
    private DcMotor elevator = null;
    //collection
    private Servo collectRight = null;
    private Servo collectLeft = null;
    private Servo push = null;
    private CRServo hold = null;
    private Servo turnHold = null;
    //moving Foundation
    private Servo grabber = null;
    private TouchSensor cubeIn = null;

    @Override
    //
    public void runOpMode()  throws InterruptedException{
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        collectRight = hardwareMap.get(Servo.class, "collectRight");
        collectLeft = hardwareMap.get(Servo.class, "collectLeft");
        push = hardwareMap.get(Servo.class, "push");
        hold = hardwareMap.get(CRServo.class, "hold");
        turnHold = hardwareMap.get(Servo.class, "turnHold");
        //grabber = hardwareMap.get(Servo.class, "grabber");
        waitForStart();
        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setDirection(DcMotor.Direction.FORWARD);
        lDrive2.setDirection(DcMotor.Direction.FORWARD);
        slide1.setDirection(DcMotor.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);

        rDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (opModeIsActive()) {

            //drive
            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                rDrive1.setPower(gamepad1.left_stick_y);
                rDrive2.setPower(gamepad1.left_stick_y);
                lDrive1.setPower(gamepad1.left_stick_y);
                lDrive2.setPower(gamepad1.left_stick_y);
            } else if (gamepad1.left_trigger > 0.2) {
                rDrive1.setPower(-gamepad1.left_trigger);
                rDrive2.setPower(-gamepad1.left_trigger);
                lDrive1.setPower(gamepad1.left_trigger);
                lDrive2.setPower(gamepad1.left_trigger);

            } else if (gamepad1.right_trigger > 0.2) {
                rDrive1.setPower(gamepad1.right_trigger);
                rDrive2.setPower(gamepad1.right_trigger);
                lDrive1.setPower(-gamepad1.right_trigger);
                lDrive2.setPower(-gamepad1.right_trigger);

            } else {
                rDrive1.setPower(0);
                rDrive2.setPower(0);
                lDrive1.setPower(0);
                lDrive2.setPower(0);
            }

            if (gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0) {
                slide1.setPower(-gamepad1.right_stick_x);
                slide2.setPower(-gamepad1.right_stick_x);
            } else {
                slide1.setPower(0);
                slide2.setPower(0);
            }


            //elevator
            if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            } else {
                elevator.setPower(0);
            }

            //collect
            if (gamepad2.right_trigger > 0) {
                collectRight.setPosition(0.7);
                collectLeft.setPosition(0.2);
            } else if (gamepad2.left_trigger > 0) {
                collectLeft.setPosition(0.7);
                collectRight.setPosition(0.2);

            } else {
                collectRight.setPosition(0);
                collectLeft.setPosition(0);
            }


            //push cube in
            if (gamepad2.x) {
                if (!pushCube) {
                    push.setPosition(180);
                    pushCube = true;
                } else {
                    push.setPosition(0);
                    pushCube = false;
                }
            }

            //pinch
            if (gamepad2.y) {
                if (!pinch) {
                    hold.setPower(-1);
                    sleep(300);
                    pinch = true;
                } else {
                    hold.setPower(1);
                    sleep(300);
                    pinch = false;
                }
            }

            //turn collection
           if (gamepad2.dpad_right){
               turnHold.setPosition(1000);
           }
           else if (gamepad2.dpad_left){
               turnHold.setPosition(0);
           }


        }

    }
}
