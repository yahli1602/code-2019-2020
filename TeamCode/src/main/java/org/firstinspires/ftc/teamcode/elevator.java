package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.TimerTask;


@TeleOp(name = "Teleop 11226 A", group = "Linear Opmode")
public class elevator extends LinearOpMode {
    private boolean pinch = false;
    private boolean pushCube = false;
    //driving motors

    //elevator
    private DcMotor elevator = null;
    //collection
    private DcMotor collectRight = null;
    private DcMotor collectLeft = null;
    private Servo pushLeft = null;
    private Servo pushRight = null;
    private CRServo hold = null;
    private CRServo turnHold = null;
    //moving Foundation
    private Servo grabber = null;
    private TouchSensor cubeIn = null;
    private boolean Fast = true;
    private PIDcon ePID = new PIDcon();
    int elevatorPosition;
    // problem fixing
    private boolean hold180 = false;
    private double fix = 0;

    class turnHP extends TimerTask {
        @Override
        public void run() {
            turnHold.setPower(1);
        }
    }


    class turnHM extends TimerTask {
        @Override
        public void run() {
            turnHold.setPower(1);
        }
    }


    turnHP turnHPlus = new turnHP();
    turnHM turnHMinus = new turnHM();


    @Override
    //
    public void runOpMode() {

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        turnHold = hardwareMap.get(CRServo.class, "turnHold");
        hold = hardwareMap.get(CRServo.class, "hold");
        pushLeft = hardwareMap.get(Servo.class, "pushLeft");
        pushRight = hardwareMap.get(Servo.class, "pushRight");
        collectRight = hardwareMap.get(DcMotor.class, "collectRight");
        collectLeft = hardwareMap.get(DcMotor.class, "collectLeft");

        pushLeft.setDirection(Servo.Direction.FORWARD);
        pushRight.setDirection(Servo.Direction.REVERSE);
        collectLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        collectRight.setDirection(DcMotorSimple.Direction.REVERSE);
        pushLeft.setPosition(0);
        pushRight.setPosition(0);
        elevator.setDirection(DcMotor.Direction.FORWARD);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();


//

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ePID.PIDcon(0.0001, 0, 0);

        while (opModeIsActive()) {


            if (gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2)
                elevator.setPower(gamepad2.left_stick_y);
            else {
                elevator.setPower(0);
            }
            telemetry.addData("elevator ticks", elevator.getCurrentPosition());
            telemetry.update();

            if (gamepad2.a) {
                pushLeft.setPosition(180);
                pushRight.setPosition(180);
            } else if (gamepad2.b) {
                pushLeft.setPosition(0.23);
                pushRight.setPosition(0.23);
            } else {
                pushRight.setPosition(0);
                pushLeft.setPosition(0);
            }

            if (gamepad2.x) {
                hold.setPower(-1);
            } else if (gamepad2.y) {
                hold.setPower(1);
            } else if (gamepad2.dpad_left) {
                hold.setPower(0);
            }


            if (gamepad2.right_bumper) {
                turnHold.setPower(-1);
            } else if (gamepad2.left_bumper) {
                turnHold.setPower(1);
            } else {
                turnHold.setPower(0);
            }

            if (gamepad2.right_trigger > 0) {
                collectLeft.setPower(1);
                collectRight.setPower(1);
            } else if (gamepad2.left_trigger > 0) {
                collectLeft.setPower(-1);
                collectRight.setPower(-1);
            } else {
                collectRight.setPower(0);
                collectLeft.setPower(0);
            }

            if (gamepad2.dpad_up) {
                setElevatorPosition(1);
            } else if (gamepad2.dpad_right) {
                setElevatorPosition(2);
            } else if (gamepad2.dpad_down) {
                setElevatorPosition(3);
            } else if (gamepad2.dpad_left) {
                setElevatorPosition(4);
            }

            telemetry.addData("dpad up", gamepad2.dpad_up);
            telemetry.addData("dpad down", gamepad2.dpad_down);
            telemetry.addData("dpad right", gamepad2.dpad_right);
            telemetry.addData("dpad left", gamepad2.dpad_left);
            telemetry.update();
            //drive





            /*if(gamepad2.b){
                setElevatorPosition(2);
                turnHold.setPower(1);
                sleep(2000);
            }*/

            //turn collection


            // Automations


        }

    }

    private boolean elevatorHasFinished = true;

    private void elevatorHight(double ticks) {
        elevatorHasFinished = false;
        telemetry.addData("elevator has finished", elevatorHasFinished);
        telemetry.update();
        ePID.setSensorValue(Math.round(elevator.getCurrentPosition() / 10));
        ePID.setSetPoint(Math.round(ticks / 10));
        ePID.setOutputRange(-0.7, 0.7);
        ePID.calculate();
        while (ePID.getError() != 0 && opModeIsActive()) {
            ePID.setSensorValue(Math.round(elevator.getCurrentPosition() / 10));
            elevator.setPower(ePID.calculate());
        }
        elevatorHasFinished = true;
        telemetry.addData("elevator has finished", elevatorHasFinished);
        telemetry.update();
    }

    private void setElevatorPosition(int ep) {
        double ticks;

        telemetry.addData("ep", ep);

        if (ep == 1) ticks = 0;
        else if (ep == 2) ticks = -820;
        else if (ep == 3) ticks = -1920;
        else if (ep == 4) ticks = -2945;
        else ticks = 0;

        elevatorHight(ticks);
    }

}
