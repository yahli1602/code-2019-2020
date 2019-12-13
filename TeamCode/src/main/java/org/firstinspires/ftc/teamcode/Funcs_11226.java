
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Funcs 11226", group = "Linear Opmode")
@Disabled
public class Funcs_11226 extends LinearOpMode {

    vuforiaFuncs11229 vuforia = new vuforiaFuncs11229();

    public ElapsedTime elapsedTime = new ElapsedTime();

    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    private DcMotor elevator = null;
    private Servo collect1 = null;
    private Servo collect2 = null;
    private Servo grabLeft = null;
    private Servo grabRight = null;

    public void init(HardwareMap HM) {

        rDrive1 = HM.get(DcMotor.class, "rDrive1");
        rDrive2 = HM.get(DcMotor.class, "rDrive2");
        lDrive1 = HM.get(DcMotor.class, "lDrive1");
        lDrive2 = HM.get(DcMotor.class, "lDrive2");
        slide = HM.get(DcMotor.class, "slide");
        grabLeft = HM.get(Servo.class, "grabLeft");
        grabRight = HM.get(Servo.class, "grabRight");

        rDrive1.setDirection(DcMotor.Direction.FORWARD);
        rDrive2.setDirection(DcMotor.Direction.FORWARD);
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);

    }

// Functions for Teleop 11226

    public void drive() {
        //Drive/turn
        if (gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2) {
            rDrive1.setPower(gamepad1.right_stick_y);
            rDrive2.setPower(gamepad1.right_stick_y);
        } else if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
            lDrive1.setPower(gamepad1.left_stick_y);
            lDrive2.setPower(gamepad1.left_stick_y);
        } else {
            rDrive1.setPower(0);
            rDrive2.setPower(0);
            lDrive1.setPower(0);
            lDrive2.setPower(0);

        }
        //slide
        if (gamepad1.right_trigger > 0) {
            slide.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0) {
            slide.setPower(gamepad1.left_trigger);
        } else {
            slide.setPower(0);
        }
    }

    public void elevator() {
        if (gamepad2.right_stick_y > 0.2) {
            elevator.setPower(gamepad2.right_stick_y);
        } else if (gamepad2.right_stick_y < 0.2) {
            elevator.setPower(gamepad2.right_stick_y);
        } else {
            elevator.setPower(0);
        }
    }

    public void timer(long miliseconds){
        long x = (long)elapsedTime.milliseconds();
        while(x < miliseconds + (long)elapsedTime.milliseconds()){}
    }

    public void grab(){
       if(gamepad2.y){
           if(grabLeft.getPosition() != 0){
               grabLeft.setPosition(0.2);
               grabRight.setPosition(0.7);
               timer(1000);
               grabLeft.setPosition(0);
               grabRight.setPosition(0);
           }
           else{
               grabLeft.setPosition(0.7);
               grabRight.setPosition(0.2);
               timer(1000);
               grabLeft.setPosition(0);
               grabRight.setPosition(0);
           }
       }
       else if (gamepad2.a){
           if(grabRight.getPosition() == 0){
               grabLeft.setPosition(0.2);
               grabRight.setPosition(0.7);
               timer(100);
               grabLeft.setPosition(0);
               grabRight.setPosition(0);

           }
           else{
               grabLeft.setPosition(0.7);
               grabRight.setPosition(0.2);
               timer(100);
               grabLeft.setPosition(0);
               grabRight.setPosition(0);
           }
       }
    }



//Functions for Autonomus

    public void rotateByDirection(String direction, float rotationPower){
        if (rDrive1.isBusy()){
            lDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive1.setPower(0);
            rDrive2.setPower(0);
        }else{
            if (direction == "right"){
                while(rotationPower > 0) {
                    lDrive1.setPower(rotationPower);
                    lDrive2.setPower(rotationPower);
                    rDrive1.setPower(-rotationPower);
                    rDrive2.setPower(-rotationPower);
                }
            }
            else if(direction == "left"){
                while(rotationPower > 0) {
                    lDrive1.setPower(-rotationPower);
                    lDrive2.setPower(-rotationPower);
                    rDrive1.setPower(rotationPower);
                    rDrive2.setPower(rotationPower);
                }
            }
        }
    }

    @Override
    public void runOpMode() {


    }
}
