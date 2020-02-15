package edu.ahs.robotics.hardware.sensors;

import java.util.List;

import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Velocity;
import edu.ahs.robotics.util.DataLogger;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.util.RingBuffer;


/**
 * A collection of Odometers used to monitor robot position. Written for Ardennes in 2019-20
 * Implementation of the class, capable of being mocked at a higher level.
 *
 * @author Alex Appleby
 */
public class OdometrySystemImpl implements OdometrySystem {
    private Position position;
    private Velocity velocity;
    private double acceleration;
    private double radius = 0; // only measured in x

    private Odometer xR, xL, y;

    private static final int DISTANCE_TIME_BUFFER_SIZE = 10;
    private RingBuffer<Double> distanceBuffer;
    private RingBuffer<Long> velocityTimeBuffer;

    private static final int SPEED_TIME_BUFFER_SIZE = 10;
    private RingBuffer<Double> speedBuffer;
    private RingBuffer<Long> accelerationTimeBuffer;

    private double distance = 0.0;
    private int loopCount = 0;

    private double dySum = 0;

    private static final int ARC_BUFFER_SIZE = 10;

    private RingBuffer<Double> x1Buffer = new RingBuffer<>(ARC_BUFFER_SIZE, 0.0);
    private RingBuffer<Double> x2Buffer = new RingBuffer<>(ARC_BUFFER_SIZE, 0.0);


    private static final int ORTHOGONAL_BUFFER_SIZE = 5;
    private RingBuffer<Double> dyBuffer;
    private RingBuffer<Long> orthogonalTimeBuffer;
    private double orthogonalVelocity;

    private double xRLast, xLLast, yLast;
    private double yInchesPerDegree;
    private double distanceBetweenYWheels;
    private Position lastPosition;

    private OdometerThread odometerThread;

    private DataLogger logger;

    /**
     * @param xR               The 'first' odometer measuring in the X direction. Should be on the right side of the robot.
     * @param xL               The 'second' odometer measuring in the X direction. Should be on the left side of the robot.
     * @param y                The odometer measuring in the Y direction.
     * @param yInchesPerDegree The amount of displacement given to the y odometer induced by a degree rotation to the robot. Tunable in OdometryCalibrationOpMode
     * @see edu.ahs.robotics.util.opmodes.OdometryCalibration
     */
    public OdometrySystemImpl(Odometer xR, Odometer xL, Odometer y, double yInchesPerDegree, double distanceBetweenYWheels) {
        this.xR = xR;
        this.xL = xL;
        this.y = y;

        position = new Position(0, 0, 0);
        velocity = Velocity.makeVelocityFromSpeedDirection(0, 0);
        lastPosition = new Position(0, 0, 0);

        this.yInchesPerDegree = yInchesPerDegree;
        this.distanceBetweenYWheels = distanceBetweenYWheels;

        odometerThread = new OdometerThread();

        logger = new DataLogger("odometryStats", "odometrySystem");

        distanceBuffer = new RingBuffer<>(DISTANCE_TIME_BUFFER_SIZE, 0.0);
        velocityTimeBuffer = new RingBuffer<>(DISTANCE_TIME_BUFFER_SIZE, 0L);//type is long

        speedBuffer = new RingBuffer<>(SPEED_TIME_BUFFER_SIZE, 0.0);
        accelerationTimeBuffer = new RingBuffer<>(SPEED_TIME_BUFFER_SIZE, 0L);

        dyBuffer = new RingBuffer<>(ORTHOGONAL_BUFFER_SIZE,0.0);
        orthogonalTimeBuffer = new RingBuffer<>(ORTHOGONAL_BUFFER_SIZE,0L);

    }

    /**
     * starts thread continuously monitoring position
     */
    public void start() {
        logger.startWriting();
        resetEncoders();
        odometerThread.start();
        logger.startWriting();
    }

    public synchronized void stop() {
        odometerThread.end();
        logger.stopWriting();
    }

    public void setPosition(double x, double y, double heading) {
        position.setPosition(x, y, heading);
        lastPosition.copyFrom(position);
    }

    @Override
    public Odometer getX1Odometer() {
        return xR;
    }

    @Override
    public Odometer getX2Odometer() {
        return xL;
    }

    /**
     * Resets encoders and sets lasts. Package Protected for testing access.
     */
    void resetEncoders() {
        xR.reset();
        xL.reset();
        y.reset();
        xRLast = xR.getDistance();
        xLLast = xL.getDistance();
        yLast = y.getDistance();
    }

    /**
     * Runs central odom math, called continuously by thread and accessible in package for unit testing
     */
    synchronized void updatePosition() {
        double xRReading, xLReading, yReading;
        double dxR, dxL, dyBeforeFactorOut, dyExpected, dy, dx;
        double dxLocal, dyLocal, dyGlobal, dxGlobal;
        double dHeading;

        long currentTime = FTCUtilities.getCurrentTimeMillis();

        //set readings from odom
        xRReading = xR.getDistance();
        xLReading = xL.getDistance();
        yReading = y.getDistance();

        FTCUtilities.OpLogger("y reading", yReading);

        //find deltas
        dxR = xRReading - xRLast;
        dxL = xLReading - xLLast;
        dx = (dxR + dxL) / 2.0; //find the average
        dyBeforeFactorOut = yReading - yLast;

        //set lasts
        xRLast = xRReading;
        xLLast = xLReading;
        yLast = yReading;

        //find change in heading
        dHeading = findDeltaHeading(dxR, dxL);

        //factor out the dy expected from rotation of robot
        dyExpected = Math.toDegrees(dHeading) * yInchesPerDegree;

        //find real dy
        dySum += dyBeforeFactorOut - dyExpected;
        dy = dyBeforeFactorOut - dyExpected;


        if (dHeading != 0) {//courtesy of 11115, thanks gluten free
            double xRadius = dx / dHeading; // arc length - l = theta*r
            double yRadius = dy / dHeading;

            //find the x and y components of each arc
            dxLocal = (xRadius * Math.sin(dHeading)) - (yRadius * (1 - Math.cos(dHeading)));
            dyLocal = (xRadius * (1 - Math.cos(dHeading))) + (yRadius * Math.sin(dHeading));


        } else { //curve with infinite radius, aka robot moves in a straight line
            dxLocal = dx;
            dyLocal = dy;
        }

        updateTravelRadius(xRReading, xLReading);
        updateOrthoganalVelocity(currentTime, dy);

        dxGlobal = -Math.sin(position.heading) * dyLocal + Math.cos(position.heading) * dxLocal; //convert to global coords. Recall that 0 rads is in direction of y axis
        dyGlobal = Math.cos(position.heading) * dyLocal + Math.sin(position.heading) * dxLocal;

        position.heading += dHeading;//apply our heading change.

        position.x += dxGlobal;
        position.y += dyGlobal;

        logger.append("xR", String.valueOf(xRReading));
        logger.append("xL", String.valueOf(xLReading));
        logger.append("yReading", String.valueOf(yReading));
        logger.append("dHeading", String.valueOf(dHeading));
        logger.append("dyBeforeFactorOut", String.valueOf(dyBeforeFactorOut));
        logger.append("dyExpected", String.valueOf(dyExpected));
        logger.append("dy", String.valueOf(dy));
        logger.append("orthogonal velocity", String.valueOf(orthogonalVelocity));


        updateVelocity(currentTime);

        //logger.append("direction of travel", String.valueOf(velocity.direction()));
        logger.append("x" , String.valueOf(position.x));
        logger.append("y" , String.valueOf(position.y));
        logger.writeLine();
    }

    /**
     * Calculates and updates a signed travel radius. Sign conveys orthoganal direction of travel. Curvature to the left is positive.
     * <a href = "https://math.stackexchange.com/questions/1216990/find-radius-of-two-concentric-arcs">Math</a>
     */
    private void updateTravelRadius(double xRReading, double xLReading) {
        double oldxR = x1Buffer.insert(xRReading);
        double oldxL = x2Buffer.insert(xLReading);

        if (loopCount++ < ARC_BUFFER_SIZE) {
            radius = 0.0;
        } else {
            double bufferDxR = xRReading - oldxR;
            double bufferDxL = xLReading - oldxL;

            double dxSum = (bufferDxR + bufferDxL);
            double arcDifference = bufferDxR - bufferDxL; // yields positive when r is big and l is small

            if(dxSum == 0.0 && arcDifference == 0.0){ //check for 0/0 case
                radius = Double.POSITIVE_INFINITY;
            } else {
                radius = (distanceBetweenYWheels * dxSum) / (2.0 * arcDifference);
            }
        }
    }

    private void updateVelocity(long currentTime) {
        double distanceTraveled = position.distanceTo(lastPosition); //distance travelled between last point and this point
        distance += distanceTraveled; // running sum of distances

        double oldDistance = distanceBuffer.insert(distance);
        long oldTime = velocityTimeBuffer.insert(currentTime);

        double deltaDistance = distance - oldDistance;
        long deltaTime = currentTime - oldTime;

        double speed = deltaDistance * 1000 / (double) deltaTime;

        double direction = lastPosition.angleTo(position);
        velocity.setVelocityFromSpeedDirection(speed, direction);

        lastPosition.copyFrom(position);

        updateAcceleration(currentTime);
    }

    private void updateOrthoganalVelocity(long currentTime, double dy){
        dyBuffer.insert(dy);
        long oldTime = orthogonalTimeBuffer.insert(currentTime);
        List<Double> allDy = dyBuffer.getBuffer();

        double totalDy = 0;
        for (double aDy : allDy){ // sum up all dy, since dy values are inherently noncumulative.
            totalDy += aDy;
        }
        double deltaTime = currentTime - oldTime; //deltaTime is double to prevent integer division

        orthogonalVelocity = (totalDy * 1000.0)/ deltaTime; //note that velocity units are in seconds
    }

    private void updateAcceleration(long currentTime){
        double oldTime = accelerationTimeBuffer.insert(currentTime);
        double oldSpeed = speedBuffer.insert(velocity.speed());

        double dv = velocity.speed() - oldSpeed;
        double dt = currentTime - oldTime;

        acceleration = dv/dt;
    }

    public synchronized State getState() {
        return new State(position, velocity, acceleration, radius, orthogonalVelocity, dySum);
    }

    public boolean isRunning() {
        return odometerThread.running;
    }


    /**
     * finds delta heading with two perpendicular odometers
     *
     * @param xR one arc distance
     * @param xL the other arc distance
     * @return the theta of the arcs in radians
     * math: https://www.desmos.com/calculator/1u5ynekr4d
     */
    private double findDeltaHeading(double xR, double xL) {
        return (xR - xL) / distanceBetweenYWheels;//derived from double arcs
    }

    private class OdometerThread extends Thread {
        private final int SLEEP_TIME = 20;

        private volatile boolean running;

        @Override
        public void run() {
            long lastTime = FTCUtilities.getCurrentTimeMillis();

            running = true;
            while (running) {
                updatePosition();

                long deltaTime = FTCUtilities.getCurrentTimeMillis() - lastTime;

                if (deltaTime < SLEEP_TIME) {
                    try {
                        synchronized (this) {
                            this.wait(SLEEP_TIME - deltaTime);
                        }
                    } catch (InterruptedException e) {
                    }
                }
                lastTime = FTCUtilities.getCurrentTimeMillis();
            }
        }

        private synchronized void end() {
            running = false;
            this.notifyAll();
        }
    }
}
