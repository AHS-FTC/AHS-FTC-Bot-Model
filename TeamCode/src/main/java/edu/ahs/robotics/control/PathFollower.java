package edu.ahs.robotics.control;

import edu.ahs.robotics.control.pid.PID;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.util.DataLogger;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.util.ParameterLookup;

public class PathFollower {
    //Amplifies negative power corrections to deal with momentum while decelerating
    private static final double DOWN_AMPLIFIER = 1; // -- tuned --
    Path path;
    DataLogger logger;
    double downCorrectionScale;
    private PID speedPID;
    private PID unifiedPID;
    private double maxPower;
    private double leftPower; //= .05;
    private double rightPower; //= .35;

    private long lastTime;

    //Correction values
    private static final double LOOK_AHEAD_TIME = 0.25; //note that this is in seconds, not millis due to speed and acceleration units.
    private static final double PID_CONSTANT_SCALAR = 0.001;// so that actual tuning values can be more fathomable to the reader

    public PathFollower(Path path, double maxPower, double leftInitialPower, double rightInitialPower) {
        this.path = path;
        this.maxPower = maxPower;

        //ParameterLookup lookup = FTCUtilities.getParameterLookup();

        //double p = lookup.getParameter("p");
        //double d = lookup.getParameter("d");

        leftPower = leftInitialPower;
        rightPower = rightInitialPower;

        speedPID = new PID(.03 * PID_CONSTANT_SCALAR, 0.0, 4 * PID_CONSTANT_SCALAR, 3); // -- tuned --
        unifiedPID = new PID(.05 * PID_CONSTANT_SCALAR, 0.0, 20 * PID_CONSTANT_SCALAR,5);

        logger = (DataLogger)Logger.getLogger("pathFollower");
        logger.startWriting();

        lastTime = FTCUtilities.getCurrentTimeMillis();
    }

    /**
     * Given a robot state, calculate speed and turning errors and update powers accordingly.
     * @param robotState
     * @return
     */
    public Powers getUpdatedPowers(OdometrySystem.State robotState) {
        long time = FTCUtilities.getCurrentTimeMillis();
        long deltaTime = time - lastTime;

        Position robotPosition = robotState.position;
        Velocity robotVelocity = robotState.velocity;

        if (!path.isFinished(robotPosition)) {

            //Find the future point where the robot will be LOOK_AHEAD_TIME in the future
            Point futurePoint = getFuturePoint(robotState, LOOK_AHEAD_TIME);
            Position futurePosition = new Position(futurePoint,0);

            //Find closest point on path to future point
            Path.Location targetLocation = path.getTargetLocation(futurePosition, 0);
            Path.Location currentLocation = path.getTargetLocation(robotPosition, 0);

            //Use PID to calculate speed correction
            PID.Corrections speedCorrections = speedPID.getCorrection(currentLocation.speed - robotVelocity.speed(), deltaTime);
            double totalSpeedCorrection = speedCorrections.totalCorrection;

            //Use a down Amplifier to increase weight on negative speed corrections
            if (totalSpeedCorrection < 0) {
                totalSpeedCorrection *= DOWN_AMPLIFIER;
            }

            leftPower += totalSpeedCorrection;
            rightPower += totalSpeedCorrection;

            //Turn error.
            //double error = targetLocation.distanceToRobot; //signed distance where positive is robot to right of path

            //Create PID controller for turning error. Positive corrections mean turn left.
            //PID.Corrections unifiedCorrections = unifiedPID.getCorrection(error,deltaTime);

            //leftPower -= unifiedCorrections.totalCorrection;
            //rightPower += unifiedCorrections.totalCorrection;


            logger.append("robotPositionX", String.valueOf(robotPosition.x));
            logger.append("robotPositionY", String.valueOf(robotPosition.y));
            logger.append("robotPositionHeading", String.valueOf(robotPosition.heading));

            //logger.append("pSpeedCorrection", String.valueOf(speedCorrections.correctionP * 1000));
            //logger.append("dSpeedCorrection", String.valueOf(speedCorrections.correctionD * 1000));
            logger.append("total speed correction", String.valueOf(speedCorrections.totalCorrection * 1000));

            logger.append("targetSpeed", String.valueOf(currentLocation.speed));
            logger.append("robotSpeed", String.valueOf(robotVelocity.speed()));

            logger.append("futureX", String.valueOf(futurePoint.x));
            logger.append("futureY", String.valueOf(futurePoint.y));

            //logger.append("p heading correction", String.valueOf(unifiedCorrections.correctionP * 1000));
            //logger.append("d heading correction", String.valueOf(unifiedCorrections.correctionD * 1000));

            //logger.append("error", String.valueOf(error));

            //logger.append("acceleration", String.valueOf(robotState.acceleration));
            logger.append("travel radius", String.valueOf(robotState.travelRadius));

            //logger.append("heading power correction", String.valueOf(unifiedCorrections.totalCorrection));

            //Clip powers to maxPower by higher power
            double higherPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (higherPower > maxPower) {
                leftPower = (leftPower / higherPower) * maxPower;
                rightPower = (rightPower / higherPower) * maxPower;
            }

            if (leftPower < .05) {
                leftPower = .05;
            }
            if (rightPower < .05) {
                rightPower = .05;
            }


        }
        //If the path is finished, stop motors
        else {
            leftPower = 0.0;
            rightPower = 0.0;
        }

        logger.append("leftPower", String.valueOf(leftPower));
        logger.append("rightPower", String.valueOf(rightPower));
        logger.append("isFinished", String.valueOf(path.isFinished(robotPosition)));
        logger.writeLine();

        lastTime = time;
        return new Powers(leftPower, rightPower, path.isFinished(robotPosition));
    }

    /**
     * Gets a projected point estimated by the current state of the robot. Useful for PID and stuff.
     * Protected for unit testing.
     * @param robotState The current state of the robot. Note that radius is signed.
     * @param lookAheadTime time in seconds to look ahead on the path.
     * @return An estimation of where the robot will be lookAheadTime seconds in the future.
     */
    /*protected for testing*/ Point getFuturePoint(OdometrySystem.State robotState, double lookAheadTime){
        double x,y;
        double futureAngleOfTravel;

        double distance = (robotState.velocity.speed() * lookAheadTime); //+ (.5) * (robotState.acceleration * (lookAheadTime * lookAheadTime)); // suvat, ut * 1/2at^2
        Vector h = Vector.makeUnitVector(robotState.directionOfTravel); //make a unit vector in the direction of heading //Was position.heading

        if(robotState.travelRadius == Double.POSITIVE_INFINITY || robotState.travelRadius == Double.NEGATIVE_INFINITY){
            h.scale(distance);

            x = robotState.position.x + h.x;
            y = robotState.position.y + h.y;

            futureAngleOfTravel = robotState.directionOfTravel; //Was position.heading
        } else {

            Vector perp = h.getPerpVector(); // Note that this is always leftward relative to robot
            perp.scale(robotState.travelRadius); // a negative radius (aka traveling right) will invert this vector rightward.

            double centerX = robotState.position.x + perp.x;
            double centerY = robotState.position.y + perp.y;

            double dx = (robotState.position.x - centerX); //effectively the unit circle components for use to derive an angle using atan2.
            double dy = (robotState.position.y - centerY);

            if(dy == 0 && dx == 0){
                x = centerX;
                y = centerY;

                futureAngleOfTravel = robotState.directionOfTravel; //Was position.heading
            } else {

                double angleToCurrentPos = Math.atan2(dy, dx);

                double angleCurrentToTarget = distance / robotState.travelRadius;//l = theta * r. Signed radius checks out, rightward angle is globally negative when added in next line

                double angleToFuturePoint = angleToCurrentPos + angleCurrentToTarget;

                x = centerX + (Math.abs(robotState.travelRadius) * Math.cos(angleToFuturePoint)); //note the absolute value on the radius
                y = centerY + (Math.abs(robotState.travelRadius) * Math.sin(angleToFuturePoint)); //since we're measuring this point relative to the center of the circle in global coords, we don't care about directionality

                futureAngleOfTravel = robotState.directionOfTravel + angleCurrentToTarget; //Was position.heading
            }
        }

        Vector futureTravel = Vector.makeUnitVector(futureAngleOfTravel);

        Vector futurePerp = futureTravel.getPerpVector(); //note that this is left 90 degrees, so positive is along local y axis
        futurePerp.scale(robotState.orthogonalVelocity * lookAheadTime); //negative velocity flips vector
        logger.append("future perp x", String.valueOf(futurePerp.x));
        logger.append("future perp y", String.valueOf(futurePerp.y));

        x += futurePerp.x;
        y += futurePerp.y;

        return new Point(x, y);
    }

    /**
     * Finds direction error ensuring appropriate angle wrapping.
     */
    private double getDirectionError(double targetDirection, double currentDirection){
        double error = targetDirection - currentDirection;

        if(error > Math.PI){
            error -= (2 * Math.PI);
        } else if (error < -Math.PI){
            error += (2 * Math.PI);
        }

        return error;
    }

    public static class Powers {
        public double leftPower;
        public double rightPower;
        public boolean pathFinished;

        public Powers(double leftPower, double rightPower, boolean pathFinished) {
            this.leftPower = leftPower;
            this.rightPower = rightPower;
            this.pathFinished = pathFinished;
        }
    }
}
