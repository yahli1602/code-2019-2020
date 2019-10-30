
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name = "Teleop gecko", group = "Linear Opmode")
public class teleopGecko extends LinearOpMode {
public DcMotor rdrive1 = null;
public DcMotor rdrive2 = null;
public DcMotor ldrive1 = null;
public DcMotor ldrive2 = null;

private double drivePower = gamepad1.right_stick_y;
private double turnPower(){
    if(gamepad1.left_trigger > 0){
        return gamepad1.left_trigger;
    }
    else if(gamepad1.right_trigger > 0){
        return -gamepad1.right_trigger;
    }
    else{
        return 0;
    }
}

public void init(HardwareMap HM){
    rdrive1 = hardwareMap.get(DcMotor.class, "right_drive1");
    rdrive2 = hardwareMap.get(DcMotor.class, "right_drive2");
    ldrive1 = hardwareMap.get(DcMotor.class,"left_drive1");
    ldrive2 = hardwareMap.get(DcMotor.class, "left_drive2");
    waitForStart();
    rdrive1.setDirection(DcMotorSimple.Direction.FORWARD);
    rdrive2.setDirection(DcMotorSimple.Direction.FORWARD);
    ldrive1.setDirection(DcMotorSimple.Direction.REVERSE);
    ldrive2.setDirection(DcMotorSimple.Direction.REVERSE);

}

    @Override
    public void runOpMode() {

        while (opModeIsActive()) {


            if (gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2){
                rdrive1.setPower(gamepad1.right_stick_y);
                rdrive2.setPower(gamepad1.right_stick_y);
                ldrive1.setPower(gamepad1.right_stick_y);
                ldrive2.setPower(gamepad1.right_stick_y);
            }

            if(drivePower > 0.2 || drivePower < -0.2){
                rdrive1.setPower(drivePower);
                rdrive2.setPower(drivePower);
                ldrive1.setPower(drivePower);
                ldrive2.setPower(drivePower);

            }else{
                rdrive1.setPower(0);
                rdrive2.setPower(0);
                ldrive1.setPower(0);
                ldrive2.setPower(0);

            }

            if (turnPower() > 0.2){
                rdrive1.setPower(turnPower());
                rdrive2.setPower(turnPower());
                ldrive1.setPower(-turnPower());
                ldrive2.setPower(-turnPower());

            }
            else if (turnPower() < -0.2){
                rdrive1.setPower(-turnPower());
                rdrive2.setPower(-turnPower());
                ldrive1.setPower(turnPower());
                ldrive2.setPower(turnPower());

            }
            else {
                rdrive1.setPower(0);
                rdrive2.setPower(0);
                ldrive1.setPower(0);
                ldrive2.setPower(0);

            }
        }
    }
}
