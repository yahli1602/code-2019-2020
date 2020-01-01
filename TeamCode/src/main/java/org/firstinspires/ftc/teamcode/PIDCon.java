package org.firstinspires.ftc.teamcode;




public class PIDCon {

    private double mKP;                     // factor for "proportional" control
    private double mKI;                     // factor for "integral" control
    private double mKD;                     // factor for "derivative" control
    private double mError = 0.0;
    private double mSetPoint;
    private double mSensorValue;
    private double mIntegral;
    private double mDerivative;
    private double mPrevError = 0.0;       // the prior sensor input (used to compute velocity)
    private double mOutput;
    private double mMaximumOutput = 1.0;	// |maximum output|
    private double mMinimumOutput = -1.0;	// |minimum output|
    private double mMaximumInput = 0.0;	// maximum input - limit setpoint to this
    private double mMinimumInput = 0.0;	// minimum input - limit setpoint to this







/*
	private boolean m_enabled = false;      // is the pid controller enabled
    private double m_input;                 // sensor input for pid controller

    private boolean m_continuous = false;	// do the endpoints wrap around? eg. Absolute encoder
    private double m_totalError = 0.0;      // the sum of the errors for use in the integral calc
    private double m_tolerance = 0.05;      // the percentage error that is considered on target
    private double m_setpoint = 0.0;
    private double m_result = 0.0;
*/

    /**
     * Allocate a PID object with the given constants for P, I, D
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient
     */
    public PIDCon(double Kp, double Ki, double Kd)
    {
        mKP = Kp;
        mKI = Ki;
        mKD = Kd;
    }

    /**
     * Read the input, calculate the output accordingly, and write to the output.
     * This should only be called by the PIDTask
     * and is created during initialization.
     */
    public double calculate()
    {
        int     sign = 1;

        // Calculate the error signal
        mError = mSetPoint - mSensorValue ;

        //calculate the integral
        // not exceed the minimum and maximum output thresholds.
        if ((Math.abs(mIntegral + mError) * mKI < mMaximumOutput) &&
                (Math.abs(mIntegral + mError) * mKI > mMinimumOutput)){
            mIntegral += mError;
        }

        //calculate the derivative
        mDerivative = mError - mPrevError;

        // Perform the primary PID calculation
        mOutput = mKP * mError + mKI * mIntegral + mKD * mDerivative;

        // Set the current error to the previous error for the next cycle.
        mPrevError = mError;

        if (mOutput < 0) sign = -1;    // Record sign of result.

        // Make sure the final result is within bounds. If we constrain the result, we make
        // sure the sign of the constrained result matches the original result sign.
        if (Math.abs(mOutput) > mMaximumOutput)
            mOutput = mMaximumOutput * sign;
        else if (Math.abs(mOutput) < mMinimumOutput)
            mOutput = mMinimumOutput * sign;
        return mOutput;

    }



    /**
     * Set the PID Controller gain parameters.
     * Set the proportional, integral, and differential coefficients.
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     *
    public void setPID(double p, double i, double d)
    {
    m_P = p;
    m_I = i;
    m_D = d;
    }*/

    /**
     * Get the Proportional coefficient
     * @return proportional coefficient
     */
    public double getP()
    {
        return mKP;
    }

    /**
     * Get the Integral coefficient
     * @return integral coefficient
     */
    public double getI()
    {
        return mKI;
    }

    /**
     * Get the Differential coefficient
     * @return differential coefficient
     */
    public double getD()
    {
        return mKD;
    }

    /**
     * Return the current PID result for the last input set with setInput().
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */

    /**
     * Return the current PID result for the specified input.
     * @param input The input value to be used to calculate the PID result.
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */


    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     * @param continuous Set to true turns on continuous, false turns off continuous
     */

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     */


    /**
     * Sets the maximum and minimum values expected from the input.
     *
     * @param minimumInput the minimum value expected from the input, always positive
     * @param maximumInput the maximum value expected from the output, always positive
     */
    public void setInputRange(double minimumInput, double maximumInput)
    {
        mMinimumInput = Math.abs(minimumInput);
        mMaximumInput = Math.abs(maximumInput);
        setSetpoint(mSetPoint);
    }

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output, always positive
     * @param maximumOutput the maximum value to write to the output, always positive
     */
    public void setOutputRange(double minimumOutput, double maximumOutput)
    {
        mMinimumOutput = Math.abs(minimumOutput);
        mMaximumOutput = Math.abs(maximumOutput);
    }

    /**
     * Set the setpoint for the PIDController
     * @param setpoint the desired setpoint
     */
    public void setSetpoint(double setpoint)
    {
        int     sign = 1;

        if (mMaximumInput > mMinimumInput)
        {
            if (setpoint < 0) sign = -1;

            if (Math.abs(setpoint) > mMaximumInput)
                mSetPoint = mMaximumInput * sign;
            else if (Math.abs(setpoint) < mMinimumInput)
                mSetPoint = mMinimumInput * sign;
            else
                mSetPoint = setpoint;
        }
        else
            mSetPoint = setpoint;
    }

    /**
     * Returns the current setpoint of the PIDController
     * @return the current setpoint
     */
    public double getSetpoint()
    {
        return mSetPoint;
    }

    /**
     * Retruns the current difference of the input from the setpoint
     * @return the current error
     */
    public synchronized double getError()
    {
        return mError;
    }

    /**
     * Set the percentage error which is considered tolerable for use with
     * OnTarget. (Input of 15.0 = 15 percent)
     * @param percent error which is tolerable
     */

    /**
     * Return true if the error is within the percentage of the total input range,
     * determined by setTolerance. This assumes that the maximum and minimum input
     * were set using setInputRange.
     * @return true if the error is less than the tolerance
     */

    /**
     * Begin running the PIDController
     */


    /**
     * Stop running the PIDController.
     */


    /**
     * Reset the previous error,, the integral term, and disable the controller.
     */
    public void reset()
    {
        mPrevError = 0;
        mIntegral = 0;
        mOutput = 0;
    }


    public void setSensorValue(double sensorValue)
    {
        int     sign = 1;

        if (mMaximumInput > mMinimumInput)
        {
            if (sensorValue < 0) sign = -1;

            if (Math.abs(sensorValue) > mMaximumInput)
                mSensorValue = mMaximumInput * sign;
            else if (Math.abs(sensorValue) < mMinimumInput)
                mSensorValue = mMinimumInput * sign;
            else
                mSensorValue = sensorValue;
        }
        else
            mSensorValue = sensorValue;
    }
}

