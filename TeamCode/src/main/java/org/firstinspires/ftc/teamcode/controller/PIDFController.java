package org.firstinspires.ftc.teamcode.controller;

/**
 * Optimized for Velocity Control (RPM)
 * Target range: 0.0 to 1.0 motor power
 */
public class PIDFController {

    private double kP, kI, kD, kF;
    private double setPoint; // This will be stored as 0-1 (Normalized)
    private double measuredValue; // This will be stored as 0-1 (Normalized)
    private double minIntegral, maxIntegral;

    private double errorVal_p;
    private double errorVal_v;

    private double totalError;
    private double prevErrorVal;

    private double errorTolerance_p = 0.01; // 1% error tolerance
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    private double lastTimeStamp;
    private double period;

    // The physical limit of your motor
    private final double MAX_VELOCITY = 6000.0;

    public PIDFController(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0, 0);
    }

    public PIDFController(double kp, double ki, double kd, double kf, double sp, double pv) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;

        // Normalize initial values
        setPoint = sp / MAX_VELOCITY;
        measuredValue = pv / MAX_VELOCITY;

        minIntegral = -1.0;
        maxIntegral = 1.0;

        lastTimeStamp = 0;
        period = 0;

        errorVal_p = setPoint - measuredValue;
        reset();
    }

    public void reset() {
        totalError = 0;
        prevErrorVal = 0;
        lastTimeStamp = 0;
    }

    /**
     * @param sp The desired RPM (0 - 6000)
     */
    public void setSetPoint(double sp) {
        setPoint = sp / MAX_VELOCITY;
        errorVal_p = setPoint - measuredValue;
    }

    /**
     * Core Calculation
     * @param pv Current measured RPM from the encoder
     * @return Motor power from 0.0 to 1.0
     */
    public double calculate(double pv) {
        prevErrorVal = errorVal_p;

        // 1. Handle Timing
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        // 2. Normalize Process Variable (RPM -> 0-1 scale)
        measuredValue = pv / MAX_VELOCITY;

        // 3. Calculate Proportional Error
        errorVal_p = setPoint - measuredValue;

        // 4. Calculate Derivative (Acceleration on 0-1 scale)
        if (Math.abs(period) > 1E-6) {
            errorVal_v = (errorVal_p - prevErrorVal) / period;
        } else {
            errorVal_v = 0;
        }

        // 5. Calculate Integral with Anti-Windup
        totalError += errorVal_p * period;
        totalError = Math.max(minIntegral, Math.min(maxIntegral, totalError));

        // 6. Calculate Output u(t)
        // Feedforward (kF) is applied to the setpoint
        double output = (kP * errorVal_p) + (kI * totalError) + (kD * errorVal_v) + (kF * setPoint);

        // 7. Clamp output to hardware limits (0 to 1)
        return Math.max(0.0, Math.min(1.0, output));
    }

    // Overload for convenience: calculate(currentRPM, targetRPM)
    public double calculate(double pv, double sp) {
        setSetPoint(sp);
        return calculate(pv);
    }

    /* Keep existing Getter/Setter methods below... */
    public void setTolerance(double positionTolerance) { errorTolerance_p = positionTolerance; }
    public boolean atSetPoint() { return Math.abs(errorVal_p) < errorTolerance_p; }
    public void setPIDF(double kp, double ki, double kd, double kf) { kP = kp; kI = ki; kD = kd; kF = kf; }
}