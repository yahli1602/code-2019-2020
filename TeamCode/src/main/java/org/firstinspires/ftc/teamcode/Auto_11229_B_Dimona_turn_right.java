
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto 11229 B Dimona right turn", group = "Autonomous")
public class Auto_11229_B_Dimona_turn_right extends LinearOpMode {

    ElapsedTime elapsedTime = new ElapsedTime();
    public DcMotor rdrive1 = null;
    public DcMotor rdrive2 = null;
    public DcMotor ldrive1 = null;
    public DcMotor ldrive2 = null;
    public DcMotor slide = null;
    public DcMotor elevator = null;



    @Override
    public void runOpMode() throws InterruptedException {
        rdrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        ldrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        rdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rdrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rdrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        rdrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rdrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        ldrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        ldrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        rdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int x = 0;
        while (opModeIsActive() && x==0) {

            rdrive1.setPower(0.7);
            rdrive2.setPower(0.7);
            ldrive1.setPower(1);
            ldrive2.setPower(1);
            sleep(600);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            sleep(100);
            rdrive1.setPower(-1);
            rdrive2.setPower(-1);
            ldrive1.setPower(1);
            ldrive2.setPower(1);
            sleep(300);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            sleep(100);
            rdrive1.setPower(0.7);
            rdrive2.setPower(0.7);
            ldrive1.setPower(1);
            ldrive2.setPower(1);
            sleep(450);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            elevator.setPower(-1);
            sleep(1000);
            elevator.setPower(1);
            sleep(1000);
            elevator.setPower(0);
            x++;
        }
    }
}




