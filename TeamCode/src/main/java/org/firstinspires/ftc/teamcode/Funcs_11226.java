
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
@Disabled
public class Funcs_11226 extends LinearOpMode {

    // Declare OpMode members.

    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    private DcMotor arm = null;
    private Servo grab = null;

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

    public void drive() {
        if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
            lDrive1.setPower(gamepad1.left_stick_y);
            lDrive2.setPower(gamepad1.left_stick_y);
        } else {
            slide.setPower(0);
            lDrive1.setPower(0);
            lDrive2.setPower(0);
        }
        if (gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2) {
            rDrive1.setPower(gamepad1.right_stick_y);
            rDrive2.setPower(gamepad1.right_stick_y);
        } else {
            slide.setPower(0);
            rDrive1.setPower(0);
            rDrive2.setPower(0);
        }
        if (gamepad1.right_trigger > 0.2) {
            slide.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.2) {
            slide.setPower(-gamepad1.left_trigger);
        } else {
            slide.setPower(0);
        }
    }


    public void arm(){
        if(gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2){
            arm.setPower(gamepad2.left_stick_y);
        }
        else{
            arm.setPower(0);
        }
    }

    public void grab(){
       if(gamepad2.y){
           if(grab.getPosition() != 0){
               grab.setPosition(0.2);
               sleep(1000);
           }
           else{
               grab.setPosition(0);
           }
       }
       else if (gamepad2.a){
           grab.setPosition(0.7);
           sleep(100);
       }
       else{}
    }

    @Override
    public void runOpMode() {


    }
}
