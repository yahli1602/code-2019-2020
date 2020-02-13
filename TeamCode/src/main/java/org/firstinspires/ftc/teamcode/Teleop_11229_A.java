package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.autonomous.PIDController.PIDcon;

@TeleOp(name = "teleop 11229 A", group = "Linear Opmode")
public class Teleop_11229_A extends LinearOpMode{
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

    private ElapsedTime time = new ElapsedTime();

    boolean fast = true;

    @Override
    public void runOpMode() throws InterruptedException{
        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "teleop_11226_A");
        collectRight = hardwareMap.get(Servo.class, "collectRight");
        collectLeft = hardwareMap.get(Servo.class, "collectLeft");
        stoneIn = hardwareMap.get(TouchSensor.class, "cubeIn");

        ePID.PIDcon(0.0001, 0, 0);


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

        while(opModeIsActive()){
            if(fast) {
                if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                    rDrive1.setPower(gamepad1.left_stick_y);
                    rDrive2.setPower(gamepad1.left_stick_y);
                    lDrive1.setPower(gamepad1.left_stick_y);
                    lDrive2.setPower(gamepad1.left_stick_y);
                } else if (gamepad1.left_trigger > 0.2) {
                    rDrive1.setPower(gamepad1.left_trigger);
                    rDrive2.setPower(gamepad1.left_trigger);
                    lDrive1.setPower(-gamepad1.left_trigger);
                    lDrive2.setPower(-gamepad1.left_trigger);
                } else if (gamepad1.right_trigger > 0.2) {
                    rDrive1.setPower(-gamepad1.right_trigger);
                    rDrive2.setPower(-gamepad1.right_trigger);
                    lDrive1.setPower(gamepad1.right_trigger);
                    lDrive2.setPower(gamepad1.right_trigger);
                } else {
                    rDrive1.setPower(0);
                    rDrive2.setPower(0);
                    lDrive1.setPower(0);
                    lDrive2.setPower(0);
                }

                if (gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < 0.2) {
                    slide.setPower(gamepad1.right_stick_x);
                } else {
                    slide.setPower(0);
                }
            }
            else{
                if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                    rDrive1.setPower(gamepad1.left_stick_y / 2);
                    rDrive2.setPower(gamepad1.left_stick_y / 2);
                    lDrive1.setPower(gamepad1.left_stick_y / 2);
                    lDrive2.setPower(gamepad1.left_stick_y / 2);
                } else if (gamepad1.left_trigger > 0.2) {
                    rDrive1.setPower(gamepad1.left_trigger / 2);
                    rDrive2.setPower(gamepad1.left_trigger / 2);
                    lDrive1.setPower(-gamepad1.left_trigger / 2);
                    lDrive2.setPower(-gamepad1.left_trigger / 2);
                } else if (gamepad1.right_trigger > 0.2) {
                    rDrive1.setPower(-gamepad1.right_trigger / 2);
                    rDrive2.setPower(-gamepad1.right_trigger / 2);
                    lDrive1.setPower(gamepad1.right_trigger / 2);
                    lDrive2.setPower(gamepad1.right_trigger / 2);
                } else {
                    rDrive1.setPower(0);
                    rDrive2.setPower(0);
                    lDrive1.setPower(0);
                    lDrive2.setPower(0);
                }

                if (gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < 0.2) {
                    slide.setPower(gamepad1.right_stick_x / 2);
                } else {
                    slide.setPower(0);
                }
            }

            if (gamepad2.a && gamepad1.atRest()) {
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

            if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            } else {
                elevator.setPower(0);
            }

            if (gamepad2.dpad_up) {
                setElevatorPosition(1);
            }
            if(gamepad2.dpad_right){
                setElevatorPosition(2);
            }

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

    private void elevatorHight(double ticks) {
        ePID.setSensorValue(elevator.getCurrentPosition());
        ePID.setSetPoint(ticks);
        ePID.setOutputRange(-0.7, 0.7);
        ePID.calculate();
        while (ePID.getError() < -5 && opModeIsActive()) {
            ePID.setSensorValue(elevator.getCurrentPosition());
            elevator.setPower(ePID.calculate());
        }
    }

    private void setElevatorPosition(int ep) {
        double ticks;

        if (ep == 1) ticks = -678;
        else if (ep == 2) ticks = -1330;
        else ticks = 0;

        elevatorHight(ticks);
    }

    private void countTime(long miliseconds) {
        long x = (long) time.milliseconds() + miliseconds;
        while (x - time.milliseconds() > 0 && opModeIsActive()) {
        }
    }

}
