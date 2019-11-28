
package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dex.EncodedValueReader;

@Autonomous(name = "Auto Gecko", group = "Autonomous")
public class Auto_11229_B extends LinearOpMode {

    ElapsedTime elapsedTime = new ElapsedTime();
    public DcMotor rdrive1 = null;
    public DcMotor rdrive2 = null;
    public DcMotor ldrive1 = null;
    public DcMotor ldrive2 = null;

    private double kp = 0.2;
    public double uT;
    private double errorT;
    private double currentPosition;
    private double lastPosition;
    private double perimeter = 4 * Math.PI;
    private double ticksPerRevolution = 1120;
    private double inchesPerTick = (perimeter / ticksPerRevolution) * 40;
    private double ticksPerSpin = ticksPerRevolution * 40;
    private double ticksPerInch = 1 / inchesPerTick;
    private int count;

    public void timer(long miliseconds) {
        long x = (long) elapsedTime.milliseconds();
        while (x < miliseconds + (long) elapsedTime.milliseconds()) {
        }
    }

    public void driveInches(double inches) {
        errorT = inches;
        lastPosition = 0;
        count = 1;
        while (errorT > 0 && opModeIsActive()) {

            currentPosition = ldrive1.getCurrentPosition() / ticksPerInch;
            errorT -= (currentPosition - lastPosition);
            lastPosition = currentPosition;

            uT = kp * errorT;
            ldrive1.setPower(uT);
            ldrive2.setPower(uT);
            rdrive1.setPower(uT);
            rdrive2.setPower(uT);

            telemetry.addData("Drive Power", ldrive1.getPower());
            telemetry.addData("ticks", ldrive1.getCurrentPosition());
            telemetry.addData("Output", uT);
            telemetry.addData("", errorT);
            telemetry.addData("count", count);
            telemetry.update();
            count++;

        }
        ldrive1.setPower(0);
        ldrive2.setPower(0);
        rdrive1.setPower(0);
        rdrive2.setPower(0);

    }
    @Override
    public void runOpMode() throws InterruptedException {
        rdrive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "right_drive2");
        ldrive1 = hardwareMap.get(DcMotor.class, "left_drive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "left_drive2");

        waitForStart();
        rdrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rdrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        ldrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        ldrive2.setDirection(DcMotorSimple.Direction.REVERSE);

        rdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            driveInches(10);

        }
    }
}




