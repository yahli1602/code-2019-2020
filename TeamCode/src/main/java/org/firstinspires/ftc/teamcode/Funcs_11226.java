
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic: Linear OpMode", group = "Linear Opmode")
public class Funcs_11226 extends LinearOpMode {

    // Declare OpMode members.

    private ElapsedTime elapsedTime = new ElapsedTime();

    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    private DcMotor arm = null;
    private Servo grab = null;

    private double rightPower = gamepad1.right_stick_y;
    private double leftPower = gamepad1.left_stick_y;
    private double slidePower(){
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
        grab = HM.get(Servo.class, "grab");

        rDrive1.setDirection(DcMotor.Direction.FORWARD);
        rDrive2.setDirection(DcMotor.Direction.FORWARD);
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

    }



    public void rotateByDirction(String direction, float rotationPower){
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
                    rDrive1.setPower(rotationPower * -1);
                    rDrive2.setPower(rotationPower * -1);
                }
            }
            else if(direction == "left"){
                while(rotationPower > 0) {
                    lDrive1.setPower(rotationPower);
                    lDrive2.setPower(rotationPower);
                    rDrive1.setPower(rotationPower * -1);
                    rDrive2.setPower(rotationPower * -1);
                }
            }
        }
    }




    public void drive() {
        if (leftPower > 0.2 || leftPower < -0.2) {
            lDrive1.setPower(leftPower);
            lDrive2.setPower(leftPower);
        } else {
            slide.setPower(0);
            lDrive1.setPower(0);
            lDrive2.setPower(0);
        }
        if (rightPower > 0.2 || rightPower < -0.2) {
            rDrive1.setPower(rightPower);
            rDrive2.setPower(rightPower);
        } else {
            slide.setPower(0);
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
           if(grab.getPosition() != 0){
               grab.setPosition(0.2);
               timer(1000);
           }
           else{
               grab.setPosition(0.7);
               timer(1000);
           }
       }
       else if (gamepad2.a){
           grab.setPosition(0.7);
           timer(100);
       }
       else{}
    }


    @Override
    public void runOpMode() {


    }
}
