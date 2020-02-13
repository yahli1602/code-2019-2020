
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.PIDController.PIDcon;


@TeleOp(name = "Teleop_11229", group = "Linear Opmode")

public class Teleop_11229 extends LinearOpMode {

    //driving motors
    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    //teleop_11226_A
    private DcMotor elevator = null;
    //collection
    private Servo collectRight = null;
    private Servo collectLeft = null;


    private TouchSensor stoneIn = null;

    private PIDcon ePID = new PIDcon();

    private int timesOfUsage = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "teleop_11226_A");
        collectRight = hardwareMap.get(Servo.class, "collectRight");
        collectLeft = hardwareMap.get(Servo.class, "collectLeft");
        stoneIn = hardwareMap.get(TouchSensor.class, "cubeIn");

        ePID.PIDcon(0.2, 0, 0);


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

            //Drive/turn tank
            if (gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2) {
                rDrive1.setPower(gamepad1.right_stick_y);
                rDrive2.setPower(gamepad1.right_stick_y);
            }

            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                lDrive1.setPower(gamepad1.left_stick_y);
                lDrive2.setPower(gamepad1.left_stick_y);
            }

//put cube on plate
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

            if (gamepad1.right_trigger > 0) {
                slide.setPower(-gamepad1.right_trigger / 2);
                telemetry.addData("Slide Power:", slide.getPower());
                telemetry.update();
            } else if (gamepad1.left_trigger > 0) {
                slide.setPower(gamepad1.left_trigger / 2);
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

//teleop_11226_A
            if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            } else {
                elevator.setPower(0);
            }

            if (gamepad2.dpad_up) {
                setElevatorPosition(1);
            }

            if (stoneIn.isPressed()) {
                telemetry.addData("Touch sensor is pressed", "the stone is inside");
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

            telemetry.addData("rticks",rDrive1.getCurrentPosition());
            telemetry.addData("lticks",lDrive1.getCurrentPosition());
            telemetry.addData("rpower",rDrive1.getPower());
            telemetry.addData("lpower",lDrive1.getPower());
            telemetry.addData("elevator ticks:", elevator.getCurrentPosition());
            telemetry.update();


        }

    }

    private void elevatorHight(double ticks) {
        double setPoint;


        ePID.setSetPoint(0);
        ePID.setOutputRange(-0.7, 0.7);
        ePID.setSensorValue(elevator.getCurrentPosition());
        ePID.calculate();
        while ((ePID.getError() <= 0 && opModeIsActive()) || (ePID.getError() >= 0 && opModeIsActive())) {
            ePID.setSensorValue(elevator.getCurrentPosition());
            elevator.setPower(ePID.calculate());
        }
    }

    private void setElevatorPosition(int ep) {
        double ticks;

        if (ep == 1) ticks = 1;
        else ticks = 0;

        elevatorHight(ticks);
    }
}