package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    vuforiaFuncs11226 vuforia = new vuforiaFuncs11226();
    ElapsedTime elapsedTime = new ElapsedTime();

    private double kp = 1.3;
    private double ki = 0.1;
    private double kd = 0.2;
    public double Ti;
    public double uT;
    public double errorL;
    public double errorT;
    public double errorN;

    private double ticksPerSpin = 1440;
    private double reduction = 40;
    private double perimeter = 4*Math.PI;
    private double ticksPerInch = ticksPerSpin * reduction / perimeter;

    public double uT(double errort){
        double count = 1;
        if(count == 1){
            errorT = errort * ticksPerInch;
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
