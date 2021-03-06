
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Teleop_11229 sensitive", group = "Linear Opmode")
@Disabled
public class Teleop_11229_sensitive extends LinearOpMode {

    //driving motors
    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    //teleop_11226_A
    private DcMotor elevator = null;
    //fold collection
    private DcMotor foldcollect = null;
    //collection
    private Servo collectRight = null;
    private Servo collectLeft = null;
    //grabbing the build plate
    private Servo grabber1 = null;
    private Servo grabber2 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "teleop_11226_A");
        foldcollect = hardwareMap.get(DcMotor.class, "foldCollect");
        collectRight = hardwareMap.get(Servo.class, "collectRight");
        collectLeft = hardwareMap.get(Servo.class, "collectLeft");
        grabber1 = hardwareMap.get(Servo.class, "grabber1");
        grabber2 = hardwareMap.get(Servo.class, "grabber2");
        waitForStart();
        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setDirection(DcMotor.Direction.FORWARD);
        lDrive2.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        foldcollect.setDirection(DcMotor.Direction.FORWARD);
        grabber1.setDirection(Servo.Direction.FORWARD);
        grabber2.setDirection(Servo.Direction.REVERSE);


        rDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (opModeIsActive()) {
            /* //Drive/turn
            if (gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2) {
                rDrive1.setPower(gamepad1.right_stick_y);
                rDrive2.setPower(gamepad1.right_stick_y);
            } else {
                rDrive1.setPower(0);
                rDrive2.setPower(0);
            }
            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                lDrive1.setPower(gamepad1.left_stick_y);
                lDrive2.setPower(gamepad1.left_stick_y);
            } else {
                lDrive1.setPower(0);
                lDrive2.setPower(0);
            }


            //slide
            if (gamepad1.right_trigger > 0) {
                slide.setPower(-gamepad1.right_trigger);
                telemetry.addData("Slide Power:", slide.getPower());
                telemetry.update();
            } else if (gamepad1.left_trigger > 0) {
                slide.setPower(gamepad1.left_trigger);
                telemetry.addData("Slide Power:", slide.getPower());
                telemetry.update();
            } else {
                slide.setPower(0);
                telemetry.addData("Slide Power:", slide.getPower());
                telemetry.update();
            }*/

             //Drive gilad
            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                if (gamepad1.left_stick_y > 0.3 && gamepad1.left_stick_y < 0.4){
                    rDrive1.setPower(gamepad1.left_stick_y - 0.1);
                    rDrive2.setPower(gamepad1.left_stick_y - 0.1);
                }else if (gamepad1.left_stick_y > 0.4 && gamepad1.left_stick_y < 0.5) {
                    rDrive1.setPower(gamepad1.left_stick_y - 0.1);
                    rDrive2.setPower(gamepad1.left_stick_y - 0.1);
                }
                if (gamepad1.left_stick_y < -0.3 && gamepad1.left_stick_y > -0.4){
                    rDrive1.setPower(gamepad1.left_stick_y + 0.1);
                    rDrive2.setPower(gamepad1.left_stick_y + 0.1);
                }else if (gamepad1.left_stick_y < -0.4 && gamepad1.left_stick_y > -0.5) {
                    rDrive1.setPower(gamepad1.left_stick_y + 0.1);
                    rDrive2.setPower(gamepad1.left_stick_y + 0.1);
                }
                if (gamepad1.left_stick_y > 0.5){
                    rDrive1.setPower(gamepad1.left_stick_y);
                    rDrive2.setPower(gamepad1.left_stick_y);
                    lDrive1.setPower(gamepad1.left_stick_y);
                    lDrive2.setPower(gamepad1.left_stick_y);
                }





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

            }

            else {
                rDrive1.setPower(0);
                rDrive2.setPower(0);
                lDrive1.setPower(0);
                lDrive2.setPower(0);
            }

            if (gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0) {
                slide.setPower(-gamepad1.right_stick_x);
            } else {
                slide.setPower(0);
            }
            telemetry.addData("rtrigger:", gamepad1.right_trigger);
            telemetry.addData("ltrigger:", gamepad1.left_trigger);
            telemetry.addData("drive:", rDrive1.getPower());
            telemetry.update();
//teleop_11226_A
            if (gamepad2.right_stick_y > 0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y < 0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            } else {
                elevator.setPower(0);
            }

            if (gamepad2.right_stick_y > 0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y < 0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            } else {
                elevator.setPower(0);
            }


            //collection
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
            //collection fold
            if (gamepad2.right_bumper) {
                foldcollect.setPower(1);
            } else if (gamepad2.left_bumper) {
                foldcollect.setPower(-1);
            } else {
                foldcollect.setPower(0);
            }

//grabber

            if (gamepad2.x) {
                grabber1.setPosition(50);
                grabber2.setPosition(50);
            } else {
                grabber1.setPosition(0);
                grabber2.setPosition(0);
            }


            //Drop cube on plate
            if(gamepad1.a){
                rDrive1.setPower(0.7);
                rDrive2.setPower(0.7);
                lDrive1.setPower(0.7);
                lDrive2.setPower(0.7);

                collectRight.setPosition(0.3);
                collectLeft.setPosition(0.7);
            }



        }


    }
}