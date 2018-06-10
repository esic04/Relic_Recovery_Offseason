package org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff;

public class PID {
    double Kp, Ki, Kd;
    double tgt, actual, lastActual;
    double Ilim, outMin, outMax;
    double errorSum;
    boolean reversed;
    boolean firstRun = true;
    double output, error, Pout, Iout, Dout;

    public PID(double Kp, double Ki, double Kd, double ILimit, double OutputMin, double OutputMax){ //set limits to 0 for no limit
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        Ilim = ILimit; //set limits to 0 to disable
        outMin = OutputMin;
        outMax = OutputMax;
    }

    public void setP(double Kp){this.Kp = Kp;}

    public void setI(double Ki){this.Ki = Ki;}

    public void setD(double Kd){this.Kd = Kd;}

    public void setPID(double Kp, double Ki, double Kd){this.Kp = Kp; this.Ki = Ki; this.Kd = Kd;}

    public void setMaxI(double maxI){Ilim = maxI;}

    public void setOutputLimits(double maxOutput, double minOutput){outMax = maxOutput; outMin = minOutput;}

    public void reversed(boolean reversed){this.reversed = reversed;}

    public void setTarget(double Target){tgt = Target;}

    public double getOutput(double actual, double target){
        tgt = target;
        this.actual = actual;

        error = tgt - actual;

        if(firstRun){
            lastActual = actual;
            firstRun = false;
        }

        Pout = Kp * error;

        Dout = -Kd * (actual - lastActual);

        Iout = Ki * errorSum;
        if(Ilim != 0){
            if(Iout > Ilim){
                Iout = Ilim;
            } else if(Iout < -Ilim){
                Iout = -Ilim;
            }
        }

        output = Pout + Iout + Dout;

        if(outMin != outMax && (output < outMin || output > outMax)){ //resets error sum if the output is too large
            errorSum = error;
        } else {
            errorSum += error;
        }

        if(outMin != outMax){
            if(output > outMax){
                output = outMax;
            } else if(output < outMin){
                output = outMin;
            }
        }

        lastActual = actual;

        return output;

    }

}
