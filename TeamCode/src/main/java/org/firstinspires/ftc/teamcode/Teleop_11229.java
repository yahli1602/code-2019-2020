
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.dex.file.ValueEncoder;


@TeleOp(name = "Teleop_11229", group = "Linear Opmode")

public class Teleop_11229 extends LinearOpMode {

    //driving motors
    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    //elevator
    private DcMotor elevator = null;
    //collection
    private Servo collectRight = null;
    private Servo collectLeft = null;

    private TouchSensor stoneIn = null;

    private boolean Fast = true;
    private double startPosition;
    private double kp = 0.3;
    private double errorT;
    private double uT;
    private double perimeter = 4 * Math.PI;
    private double ticksPerRevolution = 1120;
    private double inchesPerTick = 40 * perimeter / ticksPerRevolution;
    private double ticksPerInch = 1 / inchesPerTick;

    private void Elevator(double inches) {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        errorT = inches;
        if (inches > 0) {
            while (errorT > 0 && opModeIsActive() && elevator.getCurrentPosition() > -1120 && elevator.getCurrentPosition() < 100) {
                errorT = inches * 4 * Math.PI - Math.abs(elevator.getCurrentPosition() / ticksPerInch);
                uT = errorT * kp;
                elevator.setPower(-uT);

                telemetry.addData("uT", uT);
                telemetry.addData("power", elevator.getPower());
                telemetry.addData("errorT", errorT);
                telemetry.update();
            }
            elevator.setPower(0);
        } else {
            while (errorT < 0 && opModeIsActive()) {
                errorT = -inches - elevator.getCurrentPosition() / ticksPerInch - startPosition;
                uT = errorT * kp;
                elevator.setPower(uT);
            }
            elevator.setPower(0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        collectRight = hardwareMap.get(Servo.class, "collectRight");
        collectLeft = hardwareMap.get(Servo.class, "collectLeft");
        stoneIn = hardwareMap.get(TouchSensor.class, "cubeIn");

        waitForStart();
        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setDirection(DcMotor.Direction.FORWARD);
        lDrive2.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);


        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
//put cube on plate


            //Drive/turn tank
            if (gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2) {
                rDrive1.setPower(gamepad1.right_stick_y);
                rDrive2.setPower(gamepad1.right_stick_y);
            }

            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                lDrive1.setPower(gamepad1.left_stick_y);
                lDrive2.setPower(gamepad1.left_stick_y);
            }

            else if (gamepad2.a && gamepad1.atRest()) {
                rDrive1.setPower(0.4);
                rDrive2.setPower(0.4);
                lDrive1.setPower(0.3);
                lDrive2.setPower(0.3);
                collectRight.setPosition(0.2);
                collectLeft.setPosition(0.7);
            } else {
                rDrive1.setPower(0);
                rDrive2.setPower(0);
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


            //Drop cube on plate

            /*//Drive gilad
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

            }
            //Drop cube on plate
            else if (gamepad2.a) {
                rDrive1.setPower(0.4);
                rDrive2.setPower(0.4);
                lDrive1.setPower(0.3);
                lDrive2.setPower(0.3);
                collectRight.setPosition(0.2);
                collectLeft.setPosition(0.7);
            } else {
                rDrive1.setPower(0);
                rDrive2.setPower(0);
                lDrive1.setPower(0);
                lDrive2.setPower(0);
            }

            if (gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0) {
                slide.setPower(-gamepad1.right_stick_x);
            } else {
                slide.setPower(0);
            }*/
            telemetry.addData("rtrigger:", gamepad1.right_trigger);
            telemetry.addData("ltrigger:", gamepad1.left_trigger);
            telemetry.addData("drive:", rDrive1.getPower());
            telemetry.update();
//elevator
            if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            } else {
                elevator.setPower(0);
            }

            if (gamepad2.dpad_up) {
                Elevator(5);
            } else if (gamepad2.dpad_down) {
                Elevator(-5);
            }

            if (stoneIn.isPressed()) {
                telemetry.addData("Touch sensor is pressed", "the stone is inside");
            }

            telemetry.addData("elevator", elevator.getCurrentPosition());
            telemetry.update();

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


        }

    }
}