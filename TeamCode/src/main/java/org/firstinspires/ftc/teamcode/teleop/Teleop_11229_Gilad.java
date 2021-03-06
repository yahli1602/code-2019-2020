
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.PIDController.PIDcon;


@TeleOp(name = "Teleop_11229_Gilad", group = "Linear Opmode")

public class Teleop_11229_Gilad extends LinearOpMode {

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
    private Servo capStone = null;


    private Servo bazim = null;



    boolean fast = true;
    private PIDcon ePID = new PIDcon();


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
        bazim = hardwareMap.get(Servo.class,"bazim");
        capStone = hardwareMap.get(Servo.class, "capStone");

        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setDirection(DcMotor.Direction.FORWARD);
        lDrive2.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        collectLeft.setDirection(Servo.Direction.REVERSE);
        collectRight.setDirection(Servo.Direction.FORWARD);


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


        ePID.PIDcon(0.45,0,0);

        waitForStart();

        while (opModeIsActive()) {
            //Drive/turn tank

            //Drive gilad
            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                rDrive1.setPower(gamepad1.left_stick_y);
                rDrive2.setPower(gamepad1.left_stick_y);
                lDrive1.setPower(gamepad1.left_stick_y);
                lDrive2.setPower(gamepad1.left_stick_y);
            } else if (gamepad1.left_trigger > 0.2) {
                rDrive1.setPower(-gamepad1.left_trigger * 0.7);
                rDrive2.setPower(-gamepad1.left_trigger * 0.7);
                lDrive1.setPower(gamepad1.left_trigger * 0.7);
                lDrive2.setPower(gamepad1.left_trigger * 0.7);

            } else if (gamepad1.right_trigger > 0.2) {
                rDrive1.setPower(gamepad1.right_trigger * 0.7);
                rDrive2.setPower(gamepad1.right_trigger * 0.7);
                lDrive1.setPower(-gamepad1.right_trigger * 0.7);
                lDrive2.setPower(-gamepad1.right_trigger * 0.7);

            }
            //Drop cube on plate
            else if (gamepad2.y && gamepad1.atRest()) {
                collectRight.setPosition(0.8);
                collectLeft.setPosition(0.8);
                rDrive1.setPower(0.2);
                rDrive2.setPower(0.2);
                lDrive1.setPower(0.2);
                lDrive2.setPower(0.2);

            }
            else if (gamepad2.a && gamepad1.atRest()) {
                collectRight.setPosition(0.8);
                collectLeft.setPosition(0.8);
                rDrive1.setPower(0.3);
                rDrive2.setPower(0.3);
                lDrive1.setPower(0.3);
                lDrive2.setPower(0.3);
            } else {
                rDrive1.setPower(0);
                rDrive2.setPower(0);
                lDrive1.setPower(0);
                lDrive2.setPower(0);
            }
//slide
            if (gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < 0.2) {
                slide.setPower(gamepad1.right_stick_x);
            } else {
                slide.setPower(0);
            }

//elevator

            if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            }
            else {
                elevator.setPower(0);
            }






            //collection
            if (gamepad2.right_trigger > 0.2) {
                collectRight.setPosition(0.2);
                collectLeft.setPosition(0.2);
            } else if (gamepad2.left_trigger > 0.2) {
                collectLeft.setPosition(0.8);
                collectRight.setPosition(0.8);

            } else {
                collectRight.setPosition(0);
                collectLeft.setPosition(0);
            }

            if (gamepad2.x){
                bazim.setPosition(0.05);

            }else if (gamepad2.b){
                bazim.setPosition(0.63);
            }
            if (gamepad2.dpad_up){
                capStone.setPosition(0);
            }
            else {
                capStone.setPosition(0.9);
            }




            if (gamepad2.dpad_down) setElevatorPosition(1);

            telemetry.addData("SP",slide.getPower());
            telemetry.addData("elevator power",elevator.getPower());
            telemetry.addData("slide ticks",slide.getCurrentPosition());
            telemetry.addData("elevator ticks:", elevator.getCurrentPosition());
            telemetry.addData("bazim p",bazim.getPosition());
            telemetry.update();



        }
    }
//elh
    private void elevatorHight(double ticks) {
        ePID.setSetPoint(ticks);
        ePID.setOutputRange(-0.7, 0.7);

        ePID.setSensorValue(elevator.getCurrentPosition());
        ePID.calculate();
        while ((ePID.getError() < -20 && opModeIsActive()) || (ePID.getError() > 20 && opModeIsActive())) {
            ePID.setSensorValue(elevator.getCurrentPosition());
            elevator.setPower(ePID.calculate());
        }
    }

    private void setElevatorPosition(int ep) {
        double ticks;

        if (ep == 1) ticks = -470;

        else ticks = 0;

        elevatorHight(ticks);
    }
}