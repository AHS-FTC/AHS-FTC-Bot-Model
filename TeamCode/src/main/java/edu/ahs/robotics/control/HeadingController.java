package edu.ahs.robotics.control;

import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.sensors.Trigger;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.Logger;

public class HeadingController {
    Path path;
    Logger logger = new Logger("TestAutoData", "leftPower", "rightPower", "targetSpeed", "speedError", "distanceToRobot", "lookAheadDelta");
    private double minRampDown;
    private double minRampUp;
    private double maxVelocity;
    private double leftPower = 0;
    private double rightPower = 0;
    //Correction values
    private static final double SPEED_SCALE = .01;
    private static final double TURN_SCALE = .01;
    public static final double LOOK_AHEAD_SCALE = 0.2;

    public HeadingController(Path path, double minRampDown, double minRampUp, double maxVelocity) {
        this.path = path;
        this.minRampDown = minRampDown;
        this.minRampUp = minRampUp;
        this.maxVelocity = maxVelocity;
    }

    //P controller corrects for target point

    //I controller smoothes

    //D controller will look a number of points ahead to determine correction


    public Powers getUpdatedPowers(Position robotPosition, Velocity robotVelocity) {
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        if (!targetLocation.pathFinished) {
            double targetSpeed = getTargetSpeed(targetLocation.distanceFromStart, targetLocation.distanceToEnd);

            double speedError = targetSpeed - robotVelocity.speed;

            leftPower += speedError * SPEED_SCALE;
            rightPower += speedError * SPEED_SCALE;

            leftPower -= (targetLocation.distanceToRobot * TURN_SCALE) + (targetLocation.lookAheadDelta * LOOK_AHEAD_SCALE);
            rightPower += (targetLocation.distanceToRobot * TURN_SCALE) + (targetLocation.lookAheadDelta * LOOK_AHEAD_SCALE);

            logger.append("leftPower", String.valueOf(leftPower));
            logger.append("rightPower", String.valueOf(rightPower));
            logger.append("targetSpeed", String.valueOf(targetSpeed));
            logger.append("speedError", String.valueOf(speedError));
            logger.append("distanceToRobot", String.valueOf(targetLocation.distanceToRobot));
            logger.append("lookAheadDelta", String.valueOf(targetLocation.lookAheadDelta));

            //Clip powers to 1 by maximum power
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower = leftPower / maxPower;
                rightPower = rightPower / maxPower;
            }
        } else {
            leftPower = 0.0;
            rightPower = 0.0;
            logger.writeToFile();
        }

        return new Powers(leftPower, rightPower, targetLocation.pathFinished);
    }

    private double getTargetSpeed(double distanceFromStart, double distanceToEnd) {
        double upScale = 1; //Todo adjust
        double downScale = 1; //Todo adjust
        double rampDown = Math.max(downScale * (distanceToEnd), minRampDown);
        double rampUp = Math.max(upScale * (distanceFromStart), minRampUp);
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
