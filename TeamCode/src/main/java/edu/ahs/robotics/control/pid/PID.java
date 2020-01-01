package edu.ahs.robotics.control.pid;

/**
 * General PID class applicable for whatever the  h e c k  you want.
 * @author Alex Appleby
 */
public class PID {
    private double errorSum = 0;
    private double lastError = 0;

    private double p,i,d;

    /**
     * To disable any correction factor, use a scalar of zero.
     * @param p Proportional correction scalar
     * @param i Integral correction scalar. Generally small relative to Proportional scalar.
     * @param d Derivative scalar. Based on error derivative rather than function derivative, thus normally positive.
     */
    public PID(double p,double i,double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    /**
     * Gets a PID-derived Correction. Also logs values for integral and derivative errors.
     * @param current Current value of dependent variable
     * @param target Target Value of dependent variable
     * @return PID correction to independent variable
     */
    public double getCorrection(double current, double target){
       double correction = 0;

       double error = target - current;

       errorSum += error;

       correction += error * p;

       correction += errorSum * i;

       correction += (error - lastError)* d;

       lastError = error;

       return correction;
    }

    /**
     * Removes integral values for cases in which application changes
     */
    public void trashIntegral(){
        errorSum = 0;
    }
}
