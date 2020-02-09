package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.PIDController.PIDcon;

import java.util.TimerTask;


@TeleOp(name = "Teleop 11226", group = "Linear Opmode")
public class Teleop_11226 extends LinearOpMode {
    private boolean pinch = false;
    private boolean pushCube = false;
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

class turnHP extends TimerTask{
    @Override
    public void run() {
        turnHold.setPower(1);
    }
}


class turnHM extends TimerTask{
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
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        collectRight = hardwareMap.get(Servo.class, "collectRight");
        collectLeft = hardwareMap.get(Servo.class, "collectLeft");
        pushLeft = hardwareMap.get(Servo.class, "pushLeft");
        pushRight = hardwareMap.get(Servo.class, "pushRight");
        hold = hardwareMap.get(CRServo.class, "hold");
        turnHold = hardwareMap.get(CRServo.class, "turnHold");
        //grabber = hardwareMap.get(Servo.class, "grabber");
        waitForStart();
        rDrive1.setDirection(DcMotor.Direction.FORWARD);
        rDrive2.setDirection(DcMotor.Direction.FORWARD);
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);

        pushLeft.setPosition(0);
        pushRight.setPosition(0);

//
        rDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ePID.PIDcon(0.45,0,0);

        while (opModeIsActive()) {

            telemetry.addData("gamepad1 at rest", gamepad1.atRest());
            telemetry.addData("gamepad2 at rest", gamepad2.atRest());
            telemetry.addData("y", gamepad2.y);
            telemetry.addData("drive", gamepad1.left_stick_y);
            telemetry.addData("teleop_11226_A ticks",elevator.getCurrentPosition());
            telemetry.update();

            //drive
            if(Fast){
                if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                    rDrive1.setPower(gamepad1.left_stick_y - fix);
                    rDrive2.setPower(gamepad1.left_stick_y - fix);
                    lDrive1.setPower(gamepad1.left_stick_y - fix);
                    lDrive2.setPower(gamepad1.left_stick_y - fix);
                }
                else if (gamepad1.left_trigger > 0.2) {
                    rDrive1.setPower(gamepad1.left_trigger - fix);
                    rDrive2.setPower(gamepad1.left_trigger - fix);
                    lDrive1.setPower(-gamepad1.left_trigger + fix);
                    lDrive2.setPower(-gamepad1.left_trigger + fix);
                }
                else if (gamepad1.right_trigger > 0.2) {
                    rDrive1.setPower(-gamepad1.right_trigger + fix);
                    rDrive2.setPower(-gamepad1.right_trigger + fix);
                    lDrive1.setPower(gamepad1.right_trigger - fix);
                    lDrive2.setPower(gamepad1.right_trigger - fix);
                }
                else{
                    rDrive1.setPower(0);
                    rDrive2.setPower(0);
                    lDrive1.setPower(0);
                    lDrive2.setPower(0);
                }
            } else {
                if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                    rDrive1.setPower(gamepad1.left_stick_y / 2 - fix);
                    rDrive2.setPower(gamepad1.left_stick_y / 2 - fix);
                    lDrive1.setPower(gamepad1.left_stick_y / 2 - fix);
                    lDrive2.setPower(gamepad1.left_stick_y / 2 - fix);
                } else if (gamepad1.left_trigger > 0.2) {
                    rDrive1.setPower(gamepad1.left_trigger / 2 - fix);
                    rDrive2.setPower(gamepad1.left_trigger / 2 - fix);
                    lDrive1.setPower(-gamepad1.left_trigger / 2 + fix);
                    lDrive2.setPower(-gamepad1.left_trigger / 2 + fix);
                } else if (gamepad1.right_trigger > 0.2) {
                    rDrive1.setPower(-gamepad1.right_trigger / 2 + fix);
                    rDrive2.setPower(-gamepad1.right_trigger / 2 + fix);
                    lDrive1.setPower(gamepad1.right_trigger / 2 - fix);
                    lDrive2.setPower(gamepad1.right_trigger / 2 - fix);
                } else {
                    rDrive1.setPower(0);
                    rDrive2.setPower(0);
                    lDrive1.setPower(0);
                    lDrive2.setPower(0);
                }
            }


            while (lDrive1.isBusy()) {
                if (Fast) {
                    if ((gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) && lDrive2.getPower() != gamepad1.left_stick_y) {
                        fix = lDrive2.getPower() - gamepad1.left_stick_y;
                    } else if (gamepad1.left_trigger > 0.2 && rDrive2.getPower() != gamepad1.left_trigger) {
                        fix = rDrive2.getPower() - gamepad1.left_trigger;
                    } else if (gamepad1.right_trigger > 0.2 && lDrive2.getPower() != gamepad1.right_trigger) {
                        fix = lDrive2.getPower() - gamepad1.right_trigger;
                    } else if (lDrive2.isBusy()) {
                        fix = lDrive2.getPower();
                    }
                } else {
                    if ((gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) && lDrive2.getPower() * 2 != gamepad1.left_stick_y) {
                        fix = lDrive2.getPower() - gamepad1.left_stick_y;
                    } else if (gamepad1.left_trigger > 0.2 && rDrive2.getPower() * 2 != gamepad1.left_trigger) {
                        fix = rDrive2.getPower() - gamepad1.left_trigger;
                    } else if (gamepad1.right_trigger > 0.2 && lDrive2.getPower() * 2 != gamepad1.right_trigger) {
                        fix = lDrive2.getPower() - gamepad1.right_trigger;
                    } else if (lDrive2.isBusy()) {
                        fix = lDrive2.getPower();
                    }
                }
            }


            if (gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < 0.2) {
                slide.setPower(gamepad1.right_stick_x);
            } else {
                slide.setPower(0);
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
            if (gamepad1.a) {
                Fast = true;
            }
            if (gamepad1.b) {
                Fast = false;
            }

            //push cube in
            if (gamepad2.dpad_right) {
                pushLeft.setPosition(180);
                pushRight.setPosition(180);
            } else {
                pushLeft.setPosition(0);
                pushRight.setPosition(0);
            }


            //pinch
            if(gamepad2.x) {
                hold.setPower(1);
            }
            else if (gamepad2.y) {
                hold.setPower(-1);
            }else if (gamepad2.dpad_left){
                hold.setPower(0);
            }




            /*if(gamepad2.b){
                setElevatorPosition(2);
                turnHold.setPower(1);
                sleep(2000);
            }*/

            //turn collection
            if (gamepad2.right_bumper) {
                turnHold.setPower(1);
            } else if (gamepad2.left_bumper) {
                turnHold.setPower(-1);

            } else {
                turnHold.setPower(0);
            }
/*
            if (gamepad2.right_bumper) {
                Timer timer1 = new Timer();
                timer1.schedule(turnHPlus,2000);
                turnHold.setPower(0);
                telemetry.addData("timer has finished", Fast);
            } else if (gamepad2.left_bumper) {
                Timer timer2 = new Timer();
                timer2.schedule(turnHMinus,2000);
                turnHold.setPower(0);
                telemetry.addData("timer has finished", Fast);
            }
*/
            if (gamepad2.dpad_up) {
                setElevatorPosition(1);

            } else if (gamepad2.dpad_right) {
                setElevatorPosition(2);

            } else if (gamepad2.dpad_down) {
                setElevatorPosition(3);

            } else if (gamepad2.dpad_left) {
                setElevatorPosition(4);

            }


            // Automations



        }

    }

    private void elevatorHight(double ticks) {
        ePID.setSensorValue(elevator.getCurrentPosition());
        ePID.setSetPoint(ticks);
        ePID.setOutputRange(-0.7, 0.7);
        while (ePID.getError() != 0) {
            elevator.setPower(ePID.calculate());
        }
    }

    private void setElevatorPosition(int ep) {
        double ticks;

        if (ep == 1) ticks = 0;
        else if (ep == 2) ticks = -1542;
        else if (ep == 3) ticks = -2717;
        else if (ep == 4) ticks = -4335;
        else ticks = 0;

        elevatorHight(ticks);
    }

}
