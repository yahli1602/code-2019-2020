package org.firstinspires.ftc.teamcode.autonomous.auto11229;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class PIDcon {


    double m_error = 0;

    private double m_maximumOutput = 0;	// |maximum output|
    private double m_minimumOutput = 0;	// |minimum output|

    private double m_prevError = 0;
    private double m_startPoint = 0;
    private double m_setpoint = 0;
    private double m_sensorValue = 0;
    private double m_result = 0;

    double integral = 0;
    double derivative = 0;
    int sign;


    double m_kP = 0;
    double m_kI = 0;
    double m_kD = 0;

    public void PIDcon(double kP,double kI, double kD){
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;
    }

    public double getError(){
        return m_error;
    }

    public double getKP(){
        return m_kP;
    }

    public double getKI(){
        return m_kI;
    }

    public double getKD(){
        return m_kD;
    }

    public void setSetPoint(double setpoint){
        m_setpoint = setpoint;
    }


    public void setOutputRange(double minimumOutput, double maximumOutput){
        m_minimumOutput = minimumOutput;
        m_maximumOutput = maximumOutput;
    }


    public void setSensorValue(double sensorValue){
        m_sensorValue = sensorValue;
    }


    public void reset(){
        m_prevError = 0;
        m_error = 0;
        m_result = 0;
    }


    public double calculate(){
        sign = 1;


        m_error = m_setpoint - m_sensorValue;

        if (integral * m_kI < m_maximumOutput && integral * m_kI > m_minimumOutput){
            integral = integral + m_error;
        }

        if (m_error == 0 || m_error < m_setpoint){
            integral = 0;
        }


        //if (error is outside useful range)
        //integral = 0;
        derivative = m_error - m_prevError;
        m_prevError = m_error;
        m_result = m_error * m_kP + integral * m_kI + derivative * m_kD;

        // Record sign of result.
        if (m_result < 0) sign = -1;

        // Make sure the final result is within bounds. If we constrain the result, we make
        // sure the sign of the constrained result matches the original result sign.
        if (Math.abs(m_result) > m_maximumOutput)
            m_result = m_maximumOutput * sign;
        else if (Math.abs(m_result) < m_minimumOutput)
            m_result = m_minimumOutput * sign;

        return m_result;
    }
}
