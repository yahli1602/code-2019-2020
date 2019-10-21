package org.firstinspires.ftc.teamcode;

public class PID {

    Funcs_11226 funcs = new Funcs_11226();

    private double kp;
    private double ki;
    private double kd;
    private double Ti;
    private double uT;

    private double ticksPerSpin = 1440;
    private double reduction = 10;
    private double perimeter = 4*Math.PI;
    private double ticksPerInch = ticksPerSpin * reduction / perimeter;

    public double uT(double errorT){
        uT = 1;
        errorT *= ticksPerInch;
        double errorL = 0;
        Ti  = ki/ticksPerInch * (errorT + errorL);

        while (errorT > 0) {
            errorL += (ki/Ti) * errorT;
            Ti = funcs.elapsedTime.milliseconds() - Ti;
            uT = kp * errorT + errorL + (kd/Ti) * (errorT - errorL);
            break;
        }
        return uT;
    }
}
