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
    gecko_autonomus gecko = new gecko_autonomus();

    private double kp = 1.1;
    private double ki = 0.1;
    private double kd = 0.3;
    public double Ti;
    public double uT;
    public double errorL;
    public double errorT;
    public double errorN;


    public void driveInches(double inches){
        while(gecko.ldrive1.getCurrentPosition() < inches * 28 * 4){
            double count = 1;
            if(count == 1){
                errorT = inches;
                errorL = 0;
            }
            else{ }
            errorN = errorT;
            Ti  = ki/elapsedTime.milliseconds() * (errorN + errorL);

            errorL += (ki/Ti) * errorT;
            Ti = elapsedTime.milliseconds() - Ti;
            uT = kp * errorT + errorL + (kd/Ti) * (errorN - errorL);

            gecko.ldrive1.setPower(uT);
            gecko.ldrive2.setPower(uT);
            gecko.rdrive1.setPower(uT);
            gecko.rdrive2.setPower(uT);



            errorT -= errorL;
            count++;
        }
        gecko.ldrive1.setPower(0);
        gecko.ldrive2.setPower(0);
        gecko.rdrive1.setPower(0);
        gecko.rdrive2.setPower(0);
    }


}
