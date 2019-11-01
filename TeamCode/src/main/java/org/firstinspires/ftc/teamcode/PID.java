package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    ElapsedTime elapsedTime = new ElapsedTime();

    private double kp = 1.3;
    private double ki = 0.1;
    private double kd = 0.2;
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
        uT = kp * errorT + errorL; //+ (kd/Ti) * (errorN - errorL);

        errorT = errorL;
        errorL = errorN;
        count++;
        return uT;
    }

}
