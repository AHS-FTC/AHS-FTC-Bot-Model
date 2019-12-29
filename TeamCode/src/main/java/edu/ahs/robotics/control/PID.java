package edu.ahs.robotics.control;

/**
 * General PID class applicable for whatever the  h e c k  you want.
 * @author Alex Appleby
 */
public class PID {
    private double errorSum = 0;
    private double lastError = 0;

    private double p,i,d;

    public PID(double p,double i,double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double getCorrection(double current, double target){
       double correction = 0;

       double error = target - current;

       errorSum += error;

       correction += error * p;

       correction += errorSum * i;

       correction += (error - lastError)* d;

       return correction;
    }
}
