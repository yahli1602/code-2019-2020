package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class PID {

    ElapsedTime elapsedTime = new ElapsedTime();

    private double kp = 1.1;
    private double ki = 0.1;
    private double kd = 0.3;
    public double Ti;
    public double uT;
    public double errorL;
    public double errorT;
    public double errorN;

    public void timer(long miliseconds){
        long x = (long)elapsedTime.milliseconds();
        while(x < miliseconds + (long)elapsedTime.milliseconds()){}
    }

    public void driveInches(double inches, DcMotor ldrive1, DcMotor ldrive2, DcMotor rdrive1, DcMotor rdrive2){
        while(errorT > 0){
            if(!ldrive1.isBusy()){
                errorT = inches;
                errorL = 0;
            }
            else{ }
            errorN = errorT;
            Ti  = ki/elapsedTime.milliseconds() * (errorN + errorL);

            errorL += (ki/Ti) * errorT;
            Ti = elapsedTime.milliseconds() - Ti;
            uT = kp * errorT + errorL + (kd/Ti) * (errorN - errorL);

            ldrive1.setPower(uT);
            ldrive2.setPower(uT);
            rdrive1.setPower(uT);
            rdrive2.setPower(uT);

            errorT -= errorL;
            timer(10);
        }
        ldrive1.setPower(0);
        ldrive2.setPower(0);
        rdrive1.setPower(0);
        rdrive2.setPower(0);
    }


}
