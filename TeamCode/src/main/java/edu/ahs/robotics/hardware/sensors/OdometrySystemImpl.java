package edu.ahs.robotics.hardware.sensors;

import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Velocity;
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

    private static final int ARC_BUFFER_SIZE = 10;

    private RingBuffer<Double> x1Buffer = new RingBuffer<>(ARC_BUFFER_SIZE, 0.0);
    private RingBuffer<Double> x2Buffer = new RingBuffer<>(ARC_BUFFER_SIZE, 0.0);

    private double x1Last, x2Last, yLast;
    private double yInchesPerDegree;
    private double distanceBetweenYWheels;
    private Position lastPosition;

    private OdometerThread odometerThread;

    private Logger logger;

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

        logger = new Logger("sensorStats");

        distanceBuffer = new RingBuffer<>(DISTANCE_TIME_BUFFER_SIZE, 0.0);
        velocityTimeBuffer = new RingBuffer<>(DISTANCE_TIME_BUFFER_SIZE, 0L);//type is long

        speedBuffer = new RingBuffer<>(SPEED_TIME_BUFFER_SIZE, 0.0);
        accelerationTimeBuffer = new RingBuffer<>(SPEED_TIME_BUFFER_SIZE, 0L);
    }

    /**
     * starts thread continuously monitoring position
     */
    public void start() {
        logger.startWriting();
        resetEncoders();
        odometerThread.start();
    }

    public synchronized void stop() {
        logger.stopWriting();
        odometerThread.end();
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
        x1Last = xR.getDistance();
        x2Last = xL.getDistance();
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

        //find deltas
        dxR = xRReading - x1Last;
        dxL = xLReading - x2Last;
        dx = (dxR + dxL) / 2.0; //find the average
        dyBeforeFactorOut = yReading - yLast;

        //set lasts
        x1Last = xRReading;
        x2Last = xLReading;
        yLast = yReading;

        //find change in heading
        dHeading = findDeltaHeading(dxR, dxL);

        //factor out the dy expected from rotation of robot
        dyExpected = Math.toDegrees(dHeading) * yInchesPerDegree;

        //find real dy
        dy = dyBeforeFactorOut - dyExpected;
        //dy = 0.0; //temporary until we get y encoder


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

        position.heading += dHeading;//apply our heading change. important to do not for heading buffer.

        dxGlobal = Math.sin(position.heading) * dyLocal + Math.cos(position.heading) * dxLocal; //convert to global coords. Recall that 0 rads is in direction of y axis
        dyGlobal = Math.cos(position.heading) * dyLocal + Math.sin(position.heading) * dxLocal;

        position.x += dxGlobal;
        position.y += dyGlobal;

        //logger.append("xR", String.valueOf(x1Reading));
        //logger.append("xL", String.valueOf(x2Reading));
        //logger.append("y", String.valueOf(yReading));
        //logger.append("dHeading", String.valueOf(dHeading));
        //logger.append("dyBeforeFactorOut", String.valueOf(dyBeforeFactorOut));
        //logger.append("yFactorOut", String.valueOf(dyExpected));

        updateVelocity(currentTime);

        logger.append("direction of travel", String.valueOf(velocity.direction()));
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

    private void updateAcceleration(long currentTime){
        double oldTime = accelerationTimeBuffer.insert(currentTime);
        double oldSpeed = speedBuffer.insert(velocity.speed());

        double dv = velocity.speed() - oldSpeed;
        double dt = currentTime - oldTime;

        acceleration = dv/dt;
    }

    public State getState() { //todo make synchronized?
        return new State(position, velocity, acceleration, radius);
    }

    public boolean isRunning() {
        return odometerThread.running;
    }


    /**
     * finds delta heading with two perpendicular odometers
     *
     * @param x1 one arc distance
     * @param x2 the other arc distance
     * @return the theta of the arcs in radians
     * math: https://www.desmos.com/calculator/1u5ynekr4d
     */
    private double findDeltaHeading(double x1, double x2) {
        return (x1 - x2) / distanceBetweenYWheels;//derived from double arcs
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
                        Thread.sleep(SLEEP_TIME - deltaTime);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                lastTime = FTCUtilities.getCurrentTimeMillis();
            }
        }

        private void end() {
            running = false;
        }
    }
}
