package edu.ahs.robotics.control.pid;

import edu.ahs.robotics.control.Point;

/**
 * PID class for correcting between two 2d cartesian points on the field.
 * Based on the simple PID class.
 * @see PID
 * @author Alex Appleby
 */
public class PointPID {
    private PID xPID, yPID;

    /**
     * Full constructor that takes constants for both x and y.
     * @param xP Scalar for proportional x correction.
     * @param xI Scalar for x integral correction. Generally much smaller than xP.
     * @param xD Scalar for x derivative correction. Generally non-negative in this case.
     * @param yP Scalar for proportional y correction.
     * @param yI Scalar for y integral correction. Generally much smaller than yP.
     * @param yD Scalar for y derivative correction. Generally non-negative in this case.
     */
    public PointPID(double xP, double xI, double xD, double yP, double yI, double yD) {
        xPID = new PID(xP,xI,xD);
        yPID = new PID(yP,yI,yD);
    }

    /**
     * Simple constructor where P, I, D, constants are the same for x and y.
     * @param p Scalar for proportional correction.
     * @param i Scalar for integral correction. Generally much smaller than p.
     * @param d Scalar for derivative correction. Generally non-negative in this case.
     */
    public PointPID(double p, double i, double d){
        this(p,i,d,p,i,d);
    }

    /**
     * Gets PID corrections, also storing values for proportional and derivative math.
     * @return Correction class containing corrections in two dimensions.
     */
    public Correction getCorrection(Point current, Point target, long deltaTime){
        PID.Corrections xCorrection = xPID.getCorrection(target.x - current.x, deltaTime);
        PID.Corrections yCorrection = yPID.getCorrection(target.y - current.y, deltaTime);

        return new Correction(xCorrection.totalCorrection, yCorrection.totalCorrection);
    }

    /**
     * Contains PointPID corrections in the domain of x and y in one package.
     * Only creatable inside the PointPID Class.
     */
    public class Correction{
        public double x,y;

        private Correction(double x, double y){ //only PointPID can create corrections
            this.x = x;
            this.y = y;
        }

    }
}
