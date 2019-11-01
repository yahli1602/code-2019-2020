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

    private double kp = 1.4;
    private double ki = 0.1;
    private double kd = 0.1;
    public double Ti;
    public double uT;
    public double errorL;
    public double errorT;
    public double errorN;


    public double uT(double errort){
        double count = 1;
        if(count == 1){
            errorT = errort;
            errorL = 0;
        }
        else{}
        errorN = errorT;
        Ti  = ki/elapsedTime.milliseconds() * (errorN + errorL);

        errorL += (ki/Ti) * errorT;
        Ti = elapsedTime.milliseconds() - Ti;
        uT = kp * errorT + errorL + (kd/Ti) * (errorN - errorL);

        errorT = errorL;
        errorL = errorN;
        count++;


        return uT;
    }

    public void runOpMode(){

    }
}
