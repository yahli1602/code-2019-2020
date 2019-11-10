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

    private double kp = 0.5;
    private double ki = 0.1;
    private double kd = 0.1;
    private double Ti;
    public double uT;
    private double errorL;
    private double errorT;
    private double errorN;
    int count = 1;


    public void driveInches(double inches, DcMotor ldrive1, DcMotor ldrive2, DcMotor rdrive1, DcMotor rdrive2){
        if(inches > 0){
            while(ldrive1.getCurrentPosition() < inches * 28 * 4){
                if(count == 1){
                    errorT = inches;
                    errorL = 0;
                    count++;
                }
                else{ }
                errorN = errorT;
                Ti  = ki/elapsedTime.milliseconds() * (errorN + errorL);

                errorL += (ki/Ti) * errorT;
                uT = kp * errorT + errorL + (kd/Ti) * (errorN - errorL);

                ldrive1.setPower(uT);
                ldrive2.setPower(uT);
                rdrive1.setPower(uT);
                rdrive2.setPower(uT);

                errorT -= errorL;
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
        }
        else if (inches < 0){
            while(ldrive1.getCurrentPosition() > inches * 28 * 4){
                if(count == 1){
                    errorT = -inches;
                    errorL = 0;
                    count++;
                }
                else{ }
                errorN = errorT;
                Ti  = ki/elapsedTime.milliseconds() * (errorN + errorL);

                errorL += (ki/Ti) * errorT;
                uT = kp * errorT + errorL + (kd/Ti) * (errorN - errorL);

                ldrive1.setPower(-uT);
                ldrive2.setPower(-uT);
                rdrive1.setPower(-uT);
                rdrive2.setPower(-uT);

                errorT -= errorL;
            }
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
        }

    }
}


