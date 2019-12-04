
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name = "vuforiaTeleop11229", group = "Linear Opmode")
public class vuforiaTeleop extends LinearOpMode {
    public DcMotor rdrive1 = null;
    public DcMotor rdrive2 = null;
    public DcMotor ldrive1 = null;
    public DcMotor ldrive2 = null;
    public DcMotor slide = null;

    vuforiaFuncs11229 vuFuncs = new vuforiaFuncs11229();


    @Override
    public void runOpMode() {
        rdrive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "right_drive2");
        ldrive1 = hardwareMap.get(DcMotor.class,"left_drive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "left_drive2");
        slide = hardwareMap.get(DcMotor.class, "left_drive2");
        waitForStart();
        rdrive1.setDirection(DcMotor.Direction.FORWARD);
        rdrive2.setDirection(DcMotor.Direction.FORWARD);
        ldrive1.setDirection(DcMotor.Direction.REVERSE);
        ldrive2.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);

        rdrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rdrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        while (opModeIsActive()) {


            if (gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2 ){
                rdrive1.setPower(gamepad1.right_stick_y);
                rdrive2.setPower(gamepad1.right_stick_y);
                ldrive1.setPower(gamepad1.right_stick_y);
                ldrive2.setPower(gamepad1.right_stick_y);
            }

            if (gamepad1.left_trigger > 0.2){
                rdrive1.setPower(gamepad1.left_trigger);
                rdrive2.setPower(gamepad1.left_trigger);
                ldrive1.setPower(-gamepad1.left_trigger);
                ldrive2.setPower(-gamepad1.left_trigger);

            }
            if (gamepad1.right_trigger > 0.2){
                rdrive1.setPower(-gamepad1.right_trigger);
                rdrive2.setPower(-gamepad1.right_trigger);
                ldrive1.setPower(gamepad1.right_trigger);
                ldrive2.setPower(gamepad1.right_trigger);

            }
            else if (gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2){
                slide.setPower(gamepad1.left_stick_x);
            }

            if (gamepad1.a){
                vuFuncs.lockedOnTareget(0);
            }

            else {
                rdrive1.setPower(0);
                rdrive2.setPower(0);
                ldrive1.setPower(0);
                ldrive2.setPower(0);

            }
            telemetry.addData("", ldrive1.getCurrentPosition());
            telemetry.update();
        }
    }
}
