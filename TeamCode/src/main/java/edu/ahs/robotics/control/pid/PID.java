package edu.ahs.robotics.control.pid;

import java.util.Arrays;

import edu.ahs.robotics.util.RingBuffer;

/**
 * General PID class applicable for whatever the  h e c k  you want.
 * @author Alex Appleby and Andrew Seybold
 */
public class PID {
    private double errorSum = 0;

    private RingBuffer<Double> errorBuffer;
    private int bufferIndex = 0;

    private double p, i, d;

    /**
     * @param p Proportional correction scalar
     * @param i Integral correction scalar. Generally small relative to Proportional scalar.
     * @param d Derivative scalar. Based on error derivative rather than function derivative, thus normally positive.
     * @param errorBufferSize Set the ring buffer size to smooth out D term.
     */
    public PID(double p, double i, double d, int errorBufferSize) {
        this.p = p;
        this.i = i;
        this.d = d;

        errorBuffer = new RingBuffer<>(errorBufferSize, 0.0);
    }

    /**
     * Simple constructor, echos main constructor with an errorBuffer of 1.
     */
    public PID(double p, double i, double d) {
        this(p,i,d, 1);
    }

    /**
     * Gets a PID-derived Correction. Also stores values for integral and derivative errors.
     * @param error The amount of error to correct for. Generally measured as target - current.
     * @return PID correction to independent variable
     */
    public Corrections getCorrection(double error, long deltaTime) {
        Corrections corrections = new Corrections();

        errorSum += error;

        corrections.correctionP = error * p * deltaTime; //multiplied by deltatime to work with varying refresh rates

        corrections.correctionI = errorSum * i; //todo factor for deltaTime

        double oldError = errorBuffer.insert(error);

        corrections.correctionD = (error - oldError) * d;

        corrections.totalCorrection = corrections.correctionP + corrections.correctionI + corrections.correctionD;

        return corrections;
    }


    public static class Corrections {
        public double correctionP = 0;
        public double correctionI = 0;
        public double correctionD = 0;
        public double totalCorrection = 0;

    }

    /**
     * Removes integral values for cases in which application changes
     */
    public void trashIntegral(){
        errorSum = 0;
    }
}
