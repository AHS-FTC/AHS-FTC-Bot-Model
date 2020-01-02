package edu.ahs.robotics.control.pid;

import java.lang.reflect.Array;
import java.util.Arrays;

/**
 * General PID class applicable for whatever the  h e c k  you want.
 *
 * @author Alex Appleby
 */
public class PID {
    private double errorSum = 0;
    private double lastError = 0;

    private static final int BUFFER_SIZE = 5;
    private double[] errorBuffer;
    private int bufferIndex = 0;

    private double p, i, d;

    /**
     * To disable any correction factor, use a scalar of zero.
     *
     * @param p Proportional correction scalar
     * @param i Integral correction scalar. Generally small relative to Proportional scalar.
     * @param d Derivative scalar. Based on error derivative rather than function derivative, thus normally positive.
     */
    public PID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;

        errorBuffer = new double[BUFFER_SIZE];
        Arrays.fill(errorBuffer,0.0);
    }

    /**
     * Gets a PID-derived Correction. Also logs values for integral and derivative errors.
     *
     * @param current Current value of dependent variable
     * @param target  Target Value of dependent variable
     * @return PID correction to independent variable
     */
    public Corrections getCorrection(double current, double target) {
        Corrections corrections = new Corrections();

        double error = target - current;

        errorSum += error;

        corrections.correctionP = error * p;

        corrections.correctionI = errorSum * i;

        errorBuffer[bufferIndex] = error;
        int nextBufferIndex = nextBufferIndex();
        bufferIndex = nextBufferIndex(); //iterate bufferIndex
        corrections.correctionD = (error - errorBuffer[nextBufferIndex]) * d;

        corrections.totalCorrection = corrections.correctionP + corrections.correctionI + corrections.correctionD;

        return corrections;
    }

    private int nextBufferIndex(){
        if(bufferIndex == BUFFER_SIZE - 1){
            return 0;
        } else {
            return bufferIndex + 1;
        }
    }

    public static class Corrections {
        public double correctionP = 0;
        public double correctionI = 0;
        public double correctionD = 0;
        public double totalCorrection = 0;

    }
}
