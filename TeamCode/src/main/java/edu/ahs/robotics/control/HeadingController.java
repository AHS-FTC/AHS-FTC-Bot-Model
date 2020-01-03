package edu.ahs.robotics.control;

import edu.ahs.robotics.control.pid.PID;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.util.ParameterLookup;

public class HeadingController {
    Path path;
    Logger logger = new Logger("TestAutoData", "leftPower", "rightPower", "targetSpeed", "speedCorrection", "correctionP", "correctionI", "correctionD", "distanceToRobot", "distanceToEnd", "lookAheadTurn", "isFinished", "robotPositionX", "robotPositionY", "robotPositionHeading", "closestPointX", "closestPointY", "robotSpeed", "speedAlongPath");
    double downCorrectionScale;
    private PID speedPID;
    private double maxPower;
    private double leftPower = .6;
    private double rightPower = .6;

    //Correction values
//    private static final double SPEED_SCALE = .001;
    private static final double TURN_SCALE = .01;
    public static final double LOOK_AHEAD_SCALE = 0.2;

    public HeadingController(Path path, double maxPower) {
        this.path = path;
        this.maxPower = maxPower;

        ParameterLookup lookup = FTCUtilities.getParameterLookup();

        double pCoeff = lookup.getParameter("p");
        double dCoeff = lookup.getParameter("d");
        downCorrectionScale = lookup.getParameter("down");
        speedPID = new PID(pCoeff, 0.0, dCoeff); //i .00005, d .001

        logger.startWriting();
    }

    public Powers getUpdatedPowers(Position robotPosition, Velocity robotVelocity) {
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        if (!targetLocation.pathFinished) {
            double targetSpeed = targetLocation.lookAheadSpeed;

            double speedAlongPath = (robotVelocity.dx * targetLocation.pathDeltaX) + (robotVelocity.dy * targetLocation.pathDeltaY);
            speedAlongPath /= targetLocation.pathSegmentLength;

            PID.Corrections corrections = speedPID.getCorrection(speedAlongPath, targetSpeed);
            double speedCorrection = corrections.totalCorrection;
            if (speedCorrection < 0) {
                speedCorrection *= downCorrectionScale;
            }

            leftPower += speedCorrection;
            rightPower += speedCorrection;

            /*leftPower -= (targetLocation.distanceToRobot * TURN_SCALE) + (targetLocation.lookAheadTurn * LOOK_AHEAD_SCALE);
            rightPower += (targetLocation.distanceToRobot * TURN_SCALE) + (targetLocation.lookAheadTurn * LOOK_AHEAD_SCALE);*/

            logger.append("targetSpeed", String.valueOf(targetSpeed));
            logger.append("speedCorrection", String.valueOf(speedCorrection));
            logger.append("correctionP", String.valueOf(corrections.correctionP));
            logger.append("correctionI", String.valueOf(corrections.correctionI));
            logger.append("correctionD", String.valueOf(corrections.correctionD));
            logger.append("speedAlongPath", String.valueOf(speedAlongPath));
            logger.append("distanceToRobot", String.valueOf(targetLocation.distanceToRobot));
            logger.append("distanceToEnd", String.valueOf(targetLocation.distanceToEnd));
            logger.append("lookAheadTurn", String.valueOf(targetLocation.lookAheadTurn));
            logger.append("robotPositionX", String.valueOf(robotPosition.x));
            logger.append("robotPositionY", String.valueOf(robotPosition.y));
            logger.append("robotPositionHeading", String.valueOf(robotPosition.heading));
            logger.append("closestPointX", String.valueOf(targetLocation.closestPoint.x));
            logger.append("closestPointY", String.valueOf(targetLocation.closestPoint.y));
            logger.append("robotSpeed", String.valueOf(robotVelocity.speed()));

            //Clip powers to maxPower by higher power
            double higherPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (higherPower > maxPower) {
                leftPower = (leftPower / higherPower) * maxPower;
                rightPower = (rightPower / higherPower) * maxPower;
            }

        } else {
            leftPower = 0.0;
            rightPower = 0.0;
        }

        logger.append("leftPower", String.valueOf(leftPower));
        logger.append("rightPower", String.valueOf(rightPower));
        logger.append("isFinished", String.valueOf(targetLocation.pathFinished));
        logger.writeLine();

        if (targetLocation.pathFinished) {
            logger.stopWriting();
        }

        return new Powers(leftPower, rightPower, targetLocation.pathFinished);
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
