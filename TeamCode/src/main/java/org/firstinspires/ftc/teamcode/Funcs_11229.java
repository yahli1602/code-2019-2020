
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Funcs 11229", group = "Linear Opmode")
@Disabled
public class Funcs_11229 extends LinearOpMode {

    private ElapsedTime elapsedTime = new ElapsedTime();
    private vuforiaSkystone11229 vuforia = new vuforiaSkystone11229();

    private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    private DcMotor fourBar = null;
    private Servo collectRight = null;
    private Servo collectLeft = null;

    private double rightPower = gamepad1.right_stick_y;
    private double leftPower = gamepad1.left_stick_y;

    private double slidePower() {
        if (gamepad1.right_trigger > gamepad1.left_trigger) {
            return gamepad1.right_trigger;
        }
        return -gamepad1.left_trigger;
    }


    public void init(HardwareMap HM) {

        rDrive1 = HM.get(DcMotor.class, "rightDrive1");
        rDrive2 = HM.get(DcMotor.class, "rightDrive2");
        lDrive1 = HM.get(DcMotor.class, "leftDrive1");
        lDrive2 = HM.get(DcMotor.class, "leftDrive2");
        slide = HM.get(DcMotor.class, "slide");
        fourBar = HM.get(DcMotor.class, "4bar");
        collectRight = HM.get(Servo.class, "collectRight");
        collectLeft = HM.get(Servo.class, "collectLeft");

        rDrive1.setDirection(DcMotor.Direction.FORWARD);
        rDrive2.setDirection(DcMotor.Direction.FORWARD);
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        fourBar.setDirection(DcMotor.Direction.FORWARD);
    }
//Functions for Teleop

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

    public void timer(long miliseconds) {
        long x = (long) elapsedTime.milliseconds();
        while (x < miliseconds + (long) elapsedTime.milliseconds()) {}
    }

    public void fourBar() {
        if (gamepad2.left_stick_y > 0) {
            for (double i = 1.0; i >= 0; i -= 0.1) {
                fourBar.setPower(i);
                timer(100);
            }
        } else if (gamepad2.left_stick_y < 0) {
            for (double i = -1.0; i <= 0; i += 0.1) {
                fourBar.setPower(i);
                sleep(100);
            }
        }
    }

    public void collect() {
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
//Functions for Autonumus

    public void rotateByDirction(String direction, float rotationPower) { //rotation by direction call it again to turn off
        if (rDrive1.isBusy()) {
            lDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive1.setPower(0);
            rDrive2.setPower(0);
        } else {
            if (direction == "right") {
                while (rotationPower > 0) {
                    lDrive1.setPower(rotationPower);
                    lDrive2.setPower(rotationPower);
                    rDrive1.setPower(-rotationPower);
                    rDrive2.setPower(-rotationPower);
                }
            } else if (direction == "left") {
                while (rotationPower > 0) {
                    lDrive1.setPower(-rotationPower);
                    lDrive2.setPower(-rotationPower);
                    rDrive1.setPower(rotationPower);
                    rDrive2.setPower(rotationPower);
                }
            }
        }


    }


    public void spinUntilTarget(vuforiaSkystone11229 target) {

    }


    @Override
    public void runOpMode() {


    }
}
