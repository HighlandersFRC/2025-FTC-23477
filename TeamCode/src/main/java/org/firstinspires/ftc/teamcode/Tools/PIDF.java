package org.firstinspires.ftc.teamcode.Tools;

public class PIDF {
    private double error;
    private double totalError;
    private double prevError;

    private double PValue;
    private double IValue;
    private double DValue;
    private double FValue;

    private double maxInput;
    private double minInput;
    private double maxOutput = 1.0;
    private double minOutput = -1.0;

    // Anti-windup limits for integral term
    private double maxIntegral = 0.3;
    private double minIntegral = -0.3;

    private boolean continuous = false;
    private double setPoint;
    private double result;

    public PIDF(double kp, double ki, double kd) {
        this(kp, ki, kd, 0.0);
    }

    public PIDF(double kp, double ki, double kd, double kf) {
        PValue = kp;
        IValue = ki;
        DValue = kd;
        FValue = kf;
        totalError = 0.0;
    }

    public double updatePID(double value) {
        error = setPoint - value;

        if (continuous) {
            if (Math.abs(error) > (maxInput - minInput) / 2) {
                error = error > 0 ? error - (maxInput - minInput) : error + (maxInput - minInput);
            }
        }

        // Only accumulate integral when error is small and not saturated
        if (Math.abs(error) < 100 && result >= minOutput && result <= maxOutput) {
            totalError += error;
            // Clamp integral term to prevent windup
            if (IValue != 0) {
                totalError = Math.max(minIntegral / IValue, Math.min(maxIntegral / IValue, totalError));
            }
        } else if (Math.abs(error) > 300) {
            // Reset integral if error is very large (helps with setpoint changes)
            totalError = 0;
        }

        // Calculate output with feedforward doing most of the work
        result = PValue * error + IValue * totalError + DValue * (error - prevError) + FValue * setPoint;
        prevError = error;
        result = clamp(result);
        return result;
    }

    public void setPID(double p, double i, double d) {
        setPID(p, i, d, FValue);
    }

    public void setPID(double p, double i, double d, double f) {
        PValue = p;
        IValue = i;
        DValue = d;
        FValue = f;
    }

    public void setF(double f) {
        FValue = f;
    }

    public double getF() {
        return FValue;
    }

    public void setSetPoint(double target) {
        setPoint = target;
        // Reset integral when setpoint changes to prevent windup
        totalError = 0;
        prevError = 0;
    }

    public void reset() {
        totalError = 0;
        prevError = 0;
        error = 0;
    }

    public void setIntegralLimits(double min, double max) {
        minIntegral = min;
        maxIntegral = max;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public double getResult() {
        return result;
    }

    public void setMaxOutput(double output) {
        maxOutput = output;
    }

    public void setMinOutput(double output) {
        minOutput = output;
    }

    public void setMinInput(double input) {
        minInput = input;
    }

    public void setMaxInput(double input) {
        maxInput = input;
    }

    public void setContinuous(boolean value) {
        continuous = value;
    }

    public double clamp(double input) {
        if (input > maxOutput) {
            return maxOutput;
        }
        if (input < minOutput) {
            return minOutput;
        }
        return input;
    }

    public double getError() {
        return error;
    }
}