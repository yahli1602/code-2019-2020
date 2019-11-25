/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
public class gecko_autonomus extends LinearOpMode {

    ElapsedTime elapsedTime = new ElapsedTime();

    private double kp = 0.2;
    public double uT;
    private double errorT;
    private double currentPosition;
    private double lastPosition;
    private double perimeter = 4 * Math.PI;
    private double ticksPerRevolution = 1120 * 40;
    private double ticksPerSpin = ticksPerRevolution*perimeter;
    private double ticksPerInch = ticksPerSpin/perimeter;
    private boolean count = true;

    public void timer(long miliseconds){
        long x = (long)elapsedTime.milliseconds();
        while(x < miliseconds + (long)elapsedTime.milliseconds()){}
    }

    public void driveInches(double inches){
        if(inches > 0 && opModeIsActive()){
            errorT = inches;
            lastPosition = 0;
            while(errorT > 0 && opModeIsActive()){
                uT = kp * (errorT/ticksPerInch);

                ldrive1.setPower(uT);
                ldrive2.setPower(uT);
                rdrive1.setPower(uT);
                rdrive2.setPower(uT);

                currentPosition = ldrive1.getCurrentPosition()/ticksPerInch;
                errorT -= (currentPosition - lastPosition);
                lastPosition = currentPosition;

            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);

        }
        else{}
    }


    public DcMotor rdrive1 = null;
    public DcMotor rdrive2 = null;
    public DcMotor ldrive1 = null;
    public DcMotor ldrive2 = null;

    /*public void driveByInch(double inches) {
        for (double i = inches / 10; i > 0; i -= 0.1) {
            if (i > 1) {
                ldrive1.setPower(1);
                ldrive2.setPower(1);
                rdrive1.setPower(1);
                rdrive2.setPower(1);
            } else {
                ldrive1.setPower(i);
                ldrive2.setPower(i);
                rdrive1.setPower(i);
                rdrive2.setPower(i);
            }
        }
    }*/

    @Override
    public void runOpMode() throws InterruptedException {
        rdrive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "right_drive2");
        ldrive1 = hardwareMap.get(DcMotor.class, "left_drive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "left_drive2");

        waitForStart();
        rdrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        rdrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        ldrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        ldrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Drive Power", ldrive1.getPower());
        telemetry.update();


        while (opModeIsActive()) {
            driveInches(10);

            telemetry.addData("ticks", ldrive1.getCurrentPosition());
            telemetry.addData("power", ldrive1.getPower());
            telemetry.addData("Output", uT);
            telemetry.addData("", errorT);
            telemetry.update();
        }
    }
}