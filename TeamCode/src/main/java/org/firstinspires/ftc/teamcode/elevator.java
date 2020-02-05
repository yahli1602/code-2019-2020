package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.TimerTask;


@TeleOp(name = "Teleop 11226", group = "Linear Opmode")
public class elevator extends LinearOpMode {
    private boolean pinch = false;
    private boolean pushCube = false;
    //driving motors

    //elevator
    private DcMotor elevator = null;
    //collection
    private Servo collectRight = null;
    private Servo collectLeft = null;
    private Servo push = null;
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

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        turnHold = hardwareMap.get(CRServo.class, "turnHold");
        hold = hardwareMap.get(CRServo.class, "hold");

        waitForStart();

        elevator.setDirection(DcMotor.Direction.FORWARD);

//

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ePID.PIDcon(0.45,0,0);

        while (opModeIsActive()) {


            if (gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2) elevator.setPower(gamepad2.left_stick_y);
            else{
                elevator.setPower(0);
            }


            if (gamepad2.x){
                hold.setPower(-1);
            }else if (gamepad2.y){
                hold.setPower(1);
            }else if (gamepad2.dpad_left){
                hold.setPower(0);
            }


            if (gamepad2.right_bumper){
                turnHold.setPower(-1);
            }else if (gamepad2.left_bumper){
                turnHold.setPower(1);
            }else{
                turnHold.setPower(0);
            }





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
