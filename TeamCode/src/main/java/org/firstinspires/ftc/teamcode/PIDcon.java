package org.firstinspires.ftc.teamcode;

public class PIDcon {


    double m_error = 0;

    private double m_maximumOutput = 0.7;	// |maximum output|
    private double m_minimumOutput = -0.7;	// |minimum output|

    private double m_prevError = 0;
    private double m_startPoint = 0;
    private double m_setpoint = 0;
    private double m_sensorValue = 0;
    private double m_resulte = 0;

    double integral = 0;
    double derivative = 0;




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
        m_resulte = 0;
    }


    public double calculate(){
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
        m_resulte = m_error * m_kP + integral * m_kI + derivative * m_kD;

        if (m_resulte > m_maximumOutput) m_resulte = m_maximumOutput;


        else if (m_resulte < m_minimumOutput) m_resulte = m_minimumOutput;

        return m_resulte;
    }
}
