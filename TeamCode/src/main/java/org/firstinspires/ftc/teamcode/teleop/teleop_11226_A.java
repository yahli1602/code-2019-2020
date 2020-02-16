package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.PIDController.PIDcon;


@TeleOp(name = "Teleop 11226 A", group = "Linear Opmode")
public class teleop_11226_A extends LinearOpMode {
    private ElapsedTime time = new ElapsedTime();

    private boolean pinch = false;
    private boolean pushCube = false;
    //driving motors
    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    //elevator
    private DcMotor elevator = null;
    //collection
    private DcMotor collectRight = null;
    private DcMotor collectLeft = null;
    private Servo pushLeft = null;
    private Servo pushRight = null;
    //Pinch
    private CRServo hold = null;
    private CRServo turnHold = null;
    private Servo bazim = null;
    //moving Foundation
    private Servo grabber = null;
    private TouchSensor cubeIn = null;
    private boolean Fast = true;
    private PIDcon ePID = new PIDcon();
    int elevatorPosition;
    // problem fixing
    private boolean hold180 = false;
    private double fix = 0;
    boolean pinchDown = true;

    boolean canTimerWork = true;
    double lastEp;

    boolean pinchIn = true;
    //Time timer1 = new Time(2000);


    /*class turnHP extends TimerTask {
        @Override
        public void run() {
            turnHold.setPower(1);
        }
    }


    class turnHM extends TimerTask {
        @Override
        public void run() {
            turnHold.setPower(1);
            canTimerWork = true;
        }
    }


    turnHP turnHPlus = new turnHP();
    turnHM turnHMinus = new turnHM();


    Timer timer = new Timer();*/


    @Override
    //
    public void runOpMode() {
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        turnHold = hardwareMap.get(CRServo.class, "turnHold");
        hold = hardwareMap.get(CRServo.class, "hold");
        pushLeft = hardwareMap.get(Servo.class, "pushLeft");
        pushRight = hardwareMap.get(Servo.class, "pushRight");
        collectRight = hardwareMap.get(DcMotor.class, "collectRight");
        collectLeft = hardwareMap.get(DcMotor.class, "collectLeft");
        bazim = hardwareMap.get(Servo.class, "bazim");

        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setDirection(DcMotor.Direction.FORWARD);
        lDrive2.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        pushLeft.setDirection(Servo.Direction.FORWARD);
        pushRight.setDirection(Servo.Direction.REVERSE);
        collectLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        collectRight.setDirection(DcMotorSimple.Direction.FORWARD);
        /*pushLeft.setPosition(0);
        pushRight.setPosition(0);*/
        bazim.setDirection(Servo.Direction.FORWARD);

        bazim.setPosition(0.70);
        pushLeft.setPosition(0);
        pushRight.setPosition(0);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();


//
        rDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ePID.PIDcon(0.0001, 0, 0);

        while (opModeIsActive()) {

            //drive

            if (Fast) {
                if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                    rDrive1.setPower(gamepad1.left_stick_y - fix);
                    rDrive2.setPower(gamepad1.left_stick_y - fix);
                    lDrive1.setPower(gamepad1.left_stick_y - fix);
                    lDrive2.setPower(gamepad1.left_stick_y - fix);
                } else if (gamepad1.left_trigger > 0.2) {
                    rDrive1.setPower(-gamepad1.left_trigger*0.7 - fix);
                    rDrive2.setPower(-gamepad1.left_trigger*0.7 - fix);
                    lDrive1.setPower(gamepad1.left_trigger*0.7 + fix);
                    lDrive2.setPower(gamepad1.left_trigger*0.7 + fix);
                } else if (gamepad1.right_trigger > 0.2) {
                    rDrive1.setPower(gamepad1.right_trigger*0.7 + fix);
                    rDrive2.setPower(gamepad1.right_trigger*0.7 + fix);
                    lDrive1.setPower(-gamepad1.right_trigger*0.7 - fix);
                    lDrive2.setPower(-gamepad1.right_trigger*0.7 - fix);
                } else {
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
                    rDrive1.setPower(-gamepad1.left_trigger / 2 - fix);
                    rDrive2.setPower(-gamepad1.left_trigger / 2 - fix);
                    lDrive1.setPower(gamepad1.left_trigger / 2 + fix);
                    lDrive2.setPower(gamepad1.left_trigger / 2 + fix);
                } else if (gamepad1.right_trigger > 0.2) {
                    rDrive1.setPower(gamepad1.right_trigger / 2 + fix);
                    rDrive2.setPower(gamepad1.right_trigger / 2 + fix);
                    lDrive1.setPower(-gamepad1.right_trigger / 2 - fix);
                    lDrive2.setPower(-gamepad1.right_trigger / 2 - fix);
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



            if (gamepad1.a) {
                Fast = true;
            }
            if (gamepad1.b) {
                Fast = false;
            }





            if (gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < 0.2) {
                slide.setPower(gamepad1.right_stick_x);
            } else {
                slide.setPower(0);
            }


            if (gamepad1.dpad_left){
                bazim.setPosition(1);
            }else if (gamepad1.dpad_up){
                bazim.setPosition(0.5);
            }else if (gamepad1.dpad_down){
                bazim.setPosition(0.25);
            }else if (gamepad1.dpad_right){
                bazim.setPosition(0.75);
            }





















            if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {
                elevator.setPower(gamepad2.right_stick_y);
            }

            else {
                elevator.setPower(0);
            }

            /*if (gamepad2.right_stick_button){
                setElevatorPosition(2);
                turnPinchIn();
                setElevatorPosition(0);
            }

            if (gamepad2.dpad_up) {
                setElevatorPosition(4);

            }else if (gamepad2.dpad_down) {
                setElevatorPosition(1);


            }else if (gamepad2.a) {
                setElevatorPosition(2);


            } else if (gamepad2.b) {
                setElevatorPosition(3);

            }*/


            if (gamepad2.dpad_right) {
                pushLeft.setPosition(1);
                pushRight.setPosition(1);
            } else if (gamepad2.dpad_left) {
                pushLeft.setPosition(0);
                pushRight.setPosition(0);

            }
            else {

                pushRight.setPosition(0.215);
                pushLeft.setPosition(0.215);
            }


            if (gamepad2.y) {
                pinchDown = true;
                hold.setPower(-1);
            } else if (gamepad2.x) {
                pinchDown = false;
                hold.setPower(1);
            } else if (pinchDown) {
                hold.setPower(0);
            } else {
                hold.setPower(1);
            }


            if (gamepad2.left_bumper) {
                turnPinchIn();
            } else if (gamepad2.right_bumper) {
                turnPinchOut();
            }

            if (gamepad2.left_stick_x > 0.2) {
                turnHold.setPower(1);
            } else if (gamepad2.left_stick_x < -0.2) {
                turnHold.setPower(-1);
            } else{
                turnHold.setPower(0);
            }

            if (gamepad2.left_stick_button){
                elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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



            /*telemetry.addData("dpad up", gamepad2.dpad_up);
            telemetry.addData("dpad down", gamepad2.dpad_down);
            telemetry.addData("dpad right", gamepad2.dpad_right);
            telemetry.addData("dpad left", gamepad2.dpad_left);
            //telemetry.addData("timer",timer1.getTime());
            telemetry.update();*/


            telemetry.addData("elevator ticks",elevator.getCurrentPosition());
            telemetry.addData("push P",pushLeft.getPosition());
            telemetry.update();




            /*if(gamepad2.b){
                setElevatorPosition(2);
                turnHold.setPower(1);
                sleep(2000);
            }*/

            //turn collection


            // Automations


        }

    }

    private void elevatorHight(double ticks) {
        ePID.setSensorValue(elevator.getCurrentPosition());
        ePID.setSetPoint(ticks);
        ePID.setOutputRange(-0.7, 0.7);
        ePID.calculate();
        while ((ePID.getError() < -10 && opModeIsActive()) || (ePID.getError() > 10 && opModeIsActive())) {
            ePID.setSensorValue(elevator.getCurrentPosition());
            elevator.setPower(ePID.calculate());
        }
    }

    private void setElevatorPosition(int ep) {
        double ticks;

        if (ep == 0) ticks = 0;
        else if (ep == 1) ticks = 176;
        else if (ep == 2) ticks = -1152;
        else if (ep == 3) ticks = -2220;
        else if (ep == 4) ticks = -3060;
        else ticks = 0;

        elevatorHight(ticks);
    }

    private void countTime(long miliseconds) {
        long x = (long) time.milliseconds() + miliseconds;
        while (x - time.milliseconds() > 0 && opModeIsActive()) {
        }
    }

    private void turnPinchOut(){
        turnHold.setPower(1);
        countTime(1500);
        turnHold.setPower(0);
        pinchIn = true;
    }

    private void turnPinchIn(){
        turnHold.setPower(-1);
        countTime(1700);
        turnHold.setPower(0);
        pinchIn = false;
    }
}
