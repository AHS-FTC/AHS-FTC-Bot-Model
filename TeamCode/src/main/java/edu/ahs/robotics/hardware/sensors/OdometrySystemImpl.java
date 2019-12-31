package edu.ahs.robotics.hardware.sensors;

import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Velocity;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;


/**
 * A collection of Odometers used to monitor robot position. Written for Ardennes in 2019-20
 * Implementation of the class, capable of being mocked at a higher level.
 * @author Alex Appleby
 */
public class OdometrySystemImpl implements OdometrySystem{
    private Position position;
    private Velocity velocity;

    private Odometer x1, x2, y;

    private double x1Last, x2Last, yLast;
    private double yInchesPerDegree;
    private double distanceBetweenYWheels;
    private Position lastPosition;
    private long lastTime;

    private OdometerThread odometerThread;

    private Logger logger;

    /**
     * @param x1 The 'first' odometer measuring in the X direction. Should be interchangeable with x2
     * @param x2 The 'second' odometer measuring in the X direction. Should be interchangeable with x1
     * @param y The odometer measuring in the Y direction.
     */ //todo change axis
    public OdometrySystemImpl(Odometer x1, Odometer x2, Odometer y, double yInchesPerDegree, double distanceBetweenYWheels) {
        this.x1 = x1;
        this.x2 = x2;
        this.y = y;

        position = new Position(0,0,0);
        velocity = Velocity.makeVelocity(0,0);
        lastPosition = new Position(0,0,0);
        lastTime = FTCUtilities.getCurrentTimeMillis();

        this.yInchesPerDegree = yInchesPerDegree;
        this.distanceBetweenYWheels = distanceBetweenYWheels;

        odometerThread = new OdometerThread();

        logger = new Logger("sensorStats", "x1", "x2", "distance", "deltaTime", "speed");
    }

    /**
     * starts thread continuously monitoring position
     */
    public void start(){
        resetEncoders();
        odometerThread.start();
        logger.startWriting();
    }
  
    public void stop(){
        logger.stopWriting();
        odometerThread.end();
    }

    public void setPosition(double x, double y, double heading){
        position.setPosition(x,y,heading);
        lastPosition.copyFrom(position);
    }

    /**
     * Resets encoders and sets lasts. Package Protected for testing access.
     */
    void resetEncoders(){
        x1.reset();
        x2.reset();
        y.reset();
        x1Last = x1.getDistance();
        x2Last = x2.getDistance();
        yLast = y.getDistance();
    }

    /**
     * Runs central odom math, called continuously by thread and accessible in package for unit testing
     */
    void updatePosition() {
        double x1Reading,x2Reading, yReading;
        double dx1, dx2, dyBeforeFactorOut, dyExpected, dy, dx;
        double dxLocal, dyLocal, dyGlobal, dxGlobal;
        double dHeading;

        long currentTime = System.nanoTime(); //Calculate time immediately before getting readings from odometry

        //set readings from odom
        x1Reading = x1.getDistance();
        x2Reading = x2.getDistance();
        yReading = y.getDistance();

        FTCUtilities.addData("x1",x1Reading);
        FTCUtilities.addData("x2",x2Reading);

        //find deltas
        dx1 = x1Reading - x1Last;
        dx2 = x2Reading - x2Last;
        dx = (dx1 + dx2)/2.0; //find the average
        dyBeforeFactorOut = yReading - yLast;

        //set lasts
        x1Last = x1Reading;
        x2Last = x2Reading;
        yLast = yReading;

        //find change in heading
        dHeading = findDeltaHeading(dx1, dx2);

        //factor out the dy expected from rotation of robot
        dyExpected = Math.toDegrees(dHeading) * yInchesPerDegree;

        //find real dy
        dy = dyBeforeFactorOut - dyExpected;
        //dy = 0.0; //temporary until we get y encoder

        if(dHeading != 0){//courtesy of 11115, thanks gluten free
            double xRadius = dx/dHeading; // arc length - l = theta*r
            double yRadius = dy/dHeading;

            //find the x and y components of each arc
            dxLocal = (xRadius * Math.sin(dHeading)) - (yRadius * (1 - Math.cos(dHeading)));
            dyLocal = (xRadius * (1 - Math.cos(dHeading))) + (yRadius * Math.sin(dHeading));

        } else { //curve with infinite radius, aka robot moves in a straight line
            dxLocal = dx;
            dyLocal = dy;
        }

        position.heading += dHeading;//apply our heading change

        dxGlobal = Math.sin(position.heading)*dyLocal + Math.cos(position.heading)*dxLocal; //convert to global coords. Recall that 0 rads is in direction of y axis
        dyGlobal = Math.cos(position.heading)*dyLocal + Math.sin(position.heading)*dxLocal;

        position.x += dxGlobal;
        position.y += dyGlobal;

        logger.append("x1", String.valueOf(x1Reading));
        logger.append("x2", String.valueOf(x2Reading));


        updateVelocity(currentTime);
    }

    private void updateVelocity(long currentTime){

        double distance = position.distanceTo(lastPosition);
        double deltaTime = (currentTime - lastTime)/1.0E9;//in seconds, duh

        double speed = distance/deltaTime;
        double direction = lastPosition.angleTo(position);

        velocity.setVelocity(speed,direction);

        //lastPosition = new Position(position.x,position.y,position.heading);
        lastPosition.copyFrom(position);
        lastTime = currentTime;

        logger.append("distance", String.valueOf(distance));
        logger.append("deltaTime", String.valueOf(deltaTime));
        logger.append("speed", String.valueOf(speed));
        logger.writeLine();
    }

    public Position getPosition(){
        return position;
    }

    public Velocity getVelocity() {
        return velocity;
    }

    public boolean isRunning(){
        return odometerThread.running;
    }


    /**
     * finds delta heading with two perpendicular odometers
     * @param x1 one arc distance
     * @param x2 the other arc distance
     * @return the theta of the arcs in radians
     * math: https://www.desmos.com/calculator/1u5ynekr4d
     */
    private double findDeltaHeading(double x1, double x2){
        return (x1-x2)/distanceBetweenYWheels;//derived from double arcs
    }

    private class OdometerThread extends Thread{
        private volatile boolean running;

        @Override
        public void run() {
            running = true;
            while (running){
                updatePosition();
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {

                }
            }
        }

        public void end(){
            running = false;
        }
    }

}
