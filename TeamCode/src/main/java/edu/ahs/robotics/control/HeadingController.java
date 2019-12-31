package edu.ahs.robotics.control;

import java.util.ArrayList;

import edu.ahs.robotics.control.pid.PID;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.util.Tuner;

public class HeadingController {
    Path path;
    Logger logger = new Logger("TestAutoData", "leftPower", "rightPower", "targetSpeed", "speedCorrection", "distanceToRobot", "distanceToEnd", "lookAheadDelta", "isFinished", "robotPositionX", "robotPositionY", "robotPositionHeading", "closestPointX", "closestPointY", "robotSpeed", "actualSpeed");
    private PID speedPID;
    private double minRampDownSpeed;
    private double minRampUpSpeed;
    private double maxVelocity;
    private double maxPower;
    private double leftPower = .5;
    private double rightPower = .5;
    //Correction values
//    private static final double SPEED_SCALE = .001;
    private static final double TURN_SCALE = .01;
    public static final double LOOK_AHEAD_SCALE = 0.2;

    public HeadingController(Path path, double minRampDownSpeed, double minRampUpSpeed, double maxVelocity, double maxPower) {
        this.path = path;
        this.minRampDownSpeed = minRampDownSpeed;
        this.minRampUpSpeed = minRampUpSpeed;
        this.maxVelocity = maxVelocity;
        this.maxPower = maxPower;

        double pCoeff = Tuner.tuningParams.get(Tuner.Vals.P);
        double dCoeff = Tuner.tuningParams.get(Tuner.Vals.D);
        speedPID = new PID(pCoeff, 0.0, dCoeff); //i .00005, d .001

        logger.startWriting();
    }

    public Powers getUpdatedPowers(Position robotPosition, Velocity robotVelocity) {
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        if (!targetLocation.pathFinished) {
            double targetSpeed = getTargetSpeed(targetLocation.distanceFromStart, targetLocation.distanceToEnd);

            /*double speedError = targetSpeed - robotVelocity.speed;

            leftPower += speedError * SPEED_SCALE;
            rightPower += speedError * SPEED_SCALE;*/

            double vDeltaX = Math.cos(robotVelocity.direction) * robotVelocity.speed;
            double vDeltaY = Math.sin(robotVelocity.direction) * robotVelocity.speed;

            double actualSpeed = (vDeltaX * targetLocation.pathDeltaX) + (vDeltaY * targetLocation.pathDeltaY);
            actualSpeed /= targetLocation.pathSegmentLength;

            double speedCorrection = speedPID.getCorrection(actualSpeed, targetSpeed);
            leftPower += speedCorrection;
            rightPower += speedCorrection;

            /*leftPower -= (targetLocation.distanceToRobot * TURN_SCALE) + (targetLocation.lookAheadDelta * LOOK_AHEAD_SCALE);
            rightPower += (targetLocation.distanceToRobot * TURN_SCALE) + (targetLocation.lookAheadDelta * LOOK_AHEAD_SCALE);*/

            logger.append("targetSpeed", String.valueOf(targetSpeed));
            logger.append("speedCorrection", String.valueOf(speedCorrection));
            logger.append("actualSpeed", String.valueOf(actualSpeed));
            logger.append("distanceToRobot", String.valueOf(targetLocation.distanceToRobot));
            logger.append("distanceToEnd", String.valueOf(targetLocation.distanceToEnd));
            logger.append("lookAheadDelta", String.valueOf(targetLocation.lookAheadDelta));
            logger.append("robotPositionX", String.valueOf(robotPosition.x));
            logger.append("robotPositionY", String.valueOf(robotPosition.y));
            logger.append("robotPositionHeading", String.valueOf(robotPosition.heading));
            logger.append("closestPointX", String.valueOf(targetLocation.closestPoint.x));
            logger.append("closestPointY", String.valueOf(targetLocation.closestPoint.y));
            logger.append("robotSpeed", String.valueOf(robotVelocity.speed));

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

    private double getTargetSpeed(double distanceFromStart, double distanceToEnd) {
        double upScale = 2; //Todo adjust
        double downScale = 1; //Todo adjust
        double rampDown = Math.max(downScale * (distanceToEnd), minRampDownSpeed);
        double rampUp = Math.max(upScale * (distanceFromStart), minRampUpSpeed);
        return Math.min(rampDown, Math.min(rampUp, maxVelocity));
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
