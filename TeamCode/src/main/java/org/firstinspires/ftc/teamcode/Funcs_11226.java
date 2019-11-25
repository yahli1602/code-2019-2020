
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
    private DcMotor arm = null;
    private Servo grabLeft = null;
    private Servo grabRight = null;

    public double rightPower = gamepad1.right_stick_y;
    public double leftPower = gamepad1.left_stick_y;

    public double slidePower(){
        if(gamepad1.right_trigger > gamepad1.left_trigger){
            return gamepad1.right_trigger;
        }
        return -gamepad1.left_trigger;
    }
    private double armPower = gamepad2.left_stick_y;

    public void init(HardwareMap HM) {

        rDrive1 = HM.get(DcMotor.class, "rDrive1");
        rDrive2 = HM.get(DcMotor.class, "rDrive2");
        lDrive1 = HM.get(DcMotor.class, "lDrive1");
        lDrive2 = HM.get(DcMotor.class, "lDrive2");
        slide = HM.get(DcMotor.class, "slide");
        arm = HM.get(DcMotor.class, "arm");
        grabLeft = HM.get(Servo.class, "grabLeft");
        grabRight = HM.get(Servo.class, "grabRight");

        rDrive1.setDirection(DcMotor.Direction.FORWARD);
        rDrive2.setDirection(DcMotor.Direction.FORWARD);
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

    }

// Functions for Teleop 11226

    public void drive() {
        if (leftPower > 0.2 || leftPower < -0.2) {
            lDrive1.setPower(leftPower);
            lDrive2.setPower(leftPower);
        } else {
            lDrive1.setPower(0);
            lDrive2.setPower(0);
        }
        if (rightPower > 0.2 || rightPower < -0.2) {
            rDrive1.setPower(rightPower);
            rDrive2.setPower(rightPower);
        } else {
            rDrive1.setPower(0);
            rDrive2.setPower(0);
        }
        if (slidePower() > 0.2) {
            slide.setPower(slidePower());
        } else if (slidePower() < -0.2) {
            slide.setPower(slidePower());
        } else {
            slide.setPower(0);
        }
    }

    public void arm(){
        if(armPower > 0.2 || armPower < -0.2){
            arm.setPower(armPower);
        }
        else{
            arm.setPower(0);
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
