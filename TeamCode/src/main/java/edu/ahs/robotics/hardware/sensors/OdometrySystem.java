package edu.ahs.robotics.hardware.sensors;

import edu.ahs.robotics.autocommands.autopaths.functions.Position;

/**
 * A collection of Odometers used to monitor robot position. Written for Ardennes in 2019-20
 * @author Alex Appleby
 */
public class OdometrySystem {
    private IMU imu;
    private Position position;
    Odometer y1, y2, x;
    double y1Last, y2Last, xLast;
    double xInchesPerDegree;
    double distanceBetweenYWheels;

    private OdometerThread odometerThread;
    private Thread thread;

    /**
     * @param y1 The 'first' odometer measuring in the Y direction. Should be interchangeable with y2
     * @param y2 The 'second' odometer measuring in the Y direction. Should be interchangeable with y1
     * @param x The 'odometer measuring in the X direction.
     */
    public OdometrySystem(Odometer y1, Odometer y2, Odometer x, double xInchesPerDegree, double distanceBetweenYWheels) {
        this.y1 = y1;
        this.y2 = y2;
        this.x =x;
        position = new Position(0,0,0);
        this.xInchesPerDegree = xInchesPerDegree;
        this.distanceBetweenYWheels = distanceBetweenYWheels;

        y1Last = y1.getDistance();
        y2Last = y2.getDistance();
        xLast = x.getDistance();

        odometerThread = new OdometerThread();
        thread = new Thread(odometerThread);
    }

    /**
     * starts thread continuously monitoring position
     */
    public void start(){
        thread.start();
    }

    public void stop(){
        odometerThread.stop();
    }

    public void resetPosition(double x, double y, double heading){
        position.x = x;
        position.y = y;
        position.heading = heading;
    }

    /**
     * Runs central odom math, called continuously by thread and accessible in package for unit testing
     */
    void updatePosition() {
        double y1Reading,y2Reading, xReading;
        double dy1, dy2, dxBeforeFactorOut, dxExpected, dx, dy;
        double dyLocal, dxLocal, dyGlobal, dxGlobal;
        double dHeading;

        //set readings from odom
        y1Reading = y1.getDistance();
        y2Reading = y2.getDistance();
        xReading = x.getDistance();

        //find deltas
        dy1 = y1Reading - y1Last;
        dy2 = y2Reading - y2Last;
        dy = (dy1 + dy2)/2; //find the average
        dxBeforeFactorOut = xReading - xLast;

        //set lasts
        y1Last = y1Reading;
        y2Last = y2Reading;
        xLast = xReading;

        //find change in heading
        dHeading = findDeltaHeading(dy1, dy2);

        //factor out the dx expected from rotation of robot
        dxExpected = Math.toDegrees(dHeading) * xInchesPerDegree;

        //find real dx
        dx = dxBeforeFactorOut - dxExpected;

        if(dHeading != 0){//courtesy of 11115, thanks gluten free
            double yRadius = dy/dHeading; // arc length - l = theta*r
            double xRadius = dx/dHeading;

            //find the x and y components of each arc
            dyLocal = (yRadius * Math.sin(dHeading)) - (xRadius * (1 - Math.cos(dHeading)));
            dxLocal = (yRadius * (1 - Math.cos(dHeading))) + (xRadius * Math.sin(dHeading));

        } else { //curve with infinite radius, aka robot moves in a straight line
            dyLocal = dy;
            dxLocal = dx;
        }

        position.heading += dHeading;//apply our heading change
        double heading = position.heading; //in rads, duh

        dxGlobal = Math.sin(heading)*dxLocal + Math.cos(heading)*dyLocal; //convert to global coords. Recall that 0 rads is in direction of x axis
        dyGlobal = Math.cos(heading)*dxLocal + Math.sin(heading)*dyLocal;

        position.x += dxGlobal;
        position.y += dyGlobal;
    }

    public Position getPosition(){
        return position;
    }

    /**
     * finds delta heading with two perpendicular odometers
     * @param y1 one arc distance
     * @param y2 the other arc distance
     * @return the theta of the arcs in radians
     * math: https://www.desmos.com/calculator/1u5ynekr4d
     */
    private double findDeltaHeading(double y1, double y2){
        return (y1-y2)/distanceBetweenYWheels;//derived from double arcs
    }

    private class OdometerThread implements Runnable{
        private volatile boolean running;

        @Override
        public void run() {
            running = true;
            while (running){
                updatePosition();
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        public void stop(){
            running = false;
        }
    }

}
