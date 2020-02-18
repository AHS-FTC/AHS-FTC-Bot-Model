package edu.ahs.robotics.util;

import java.util.List;
import static java.lang.Math.*; //we need lots of these

/**
 * Contains useful static math methods for general FTC use.
 * @author Alex Appleby
 */
public class FTCMath {
    /**
     * Wraps an angle to the range -2pi to 2pi
     * @param angle Angle in radians
     */
    public static double wrapAngle(double angle){
        return angle % (2 * PI);
    }

    /**
     * Ensures that an angle is expressed in the shortest distance from 0 radians as possible. Generally useful for angle differences.
     * @param angle Angle in radians.
     */
    public static double ensureIdealAngle(double angle){
        while (angle > PI){
            angle -= (2 * PI);
        }
        while (angle < - PI){
            angle += (2 * PI);
        }
        return angle;
    }

    /**
     * Finds the mean of a RingBuffer<Double>. Does not work with ints, longs, etc.
     */
    public static double ringBufferMean(RingBuffer<Double> ringBuffer){
        List<Double> buffer = ringBuffer.getBuffer();

        double sum = 0.0;

        for(Double d : buffer){
            sum += d;
        }

        return sum / (double)buffer.size();
    }

    /**
     * Calculates the standard deviation of a RingBuffer<Double>. Does not work with ints, longs, etc.
     * <a href = "https://www.mathsisfun.com/data/standard-deviation-formulas.html">Math</>
     */
    public static double ringBufferStandardDeviation(RingBuffer<Double> ringBuffer){
        List<Double> buffer = ringBuffer.getBuffer();

        double mean = ringBufferMean(ringBuffer);

        double sum = 0;

        for(Double d : buffer){
          double distanceToMean = d - mean; //negative doesn't matter

          double distanceToMeanSquared = distanceToMean * distanceToMean;

          sum += distanceToMeanSquared;
        }

        double dividedSum = sum / (double)(buffer.size() - 1);//divide by n - 1 for reasons. IDK don't ask me.

        return sqrt(dividedSum);
    }
}
