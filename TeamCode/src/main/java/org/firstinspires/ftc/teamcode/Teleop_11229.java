
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.dex.file.ValueEncoder;


@TeleOp(name = "Teleop_11229", group = "Linear Opmode")

public class Teleop_11229 extends LinearOpMode {
//private Funcs_11229 funcs = new Funcs_11229();

    //driving motors
    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    //elevator
    private DcMotor elevator = null;
    //fold collection
    private DcMotor foldcollect = null;
    //collection
    private Servo collectRight = null;
    private Servo collectLeft = null;
    //grabbing the build plate
    private Servo grabber = null;

    @Override
    public void runOpMode() throws InterruptedException {
        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        // foldcollect = hardwareMap.get(DcMotor.class, "foldCollect");
        //collectRight = hardwareMap.get(Servo.class, "collectRight");
        //collectLeft = hardwareMap.get(Servo.class, "collectLeft");
        //grabber = hardwareMap.get(Servo.class, "grabber");
        waitForStart();
        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setDirection(DcMotor.Direction.FORWARD);
        lDrive2.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);
//        foldcollect.setDirection(DcMotor.Direction.FORWARD);


        {


            while (opModeIsActive()) {
                //Drive/turn
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
                }

//elevator
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
            boolean grabbervar = true;
            if (gamepad2.x) {
                if (grabbervar) {
                    grabber.setPosition(180);
                    grabbervar = false;
                } else {
                    grabber.setPosition(0);
                    grabbervar = true;
                }
            }


            telemetry.addData("ticks", lDrive1.getCurrentPosition());
            telemetry.update();


        }
    }


}