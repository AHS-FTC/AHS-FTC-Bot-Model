package edu.ahs.robotics.control;

import edu.ahs.robotics.control.pid.PID;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.util.ParameterLookup;

public class HeadingController {
    //Amplifies negative power corrections to deal with momentum while decelerating
    private static final double DOWN_AMPLIFIER = 1.2;
    Path path;
    Logger logger = new Logger("TestAutoData", "leftPower", "rightPower", "targetSpeed", "speedCorrection", "speedCorrectionP", "speedCorrectionI", "speedCorrectionD", "distanceToRobot", "distanceToEnd", "lookAheadTurn", "isFinished", "robotPositionX", "robotPositionY", "robotPositionHeading", "closestPointX", "closestPointY", "robotSpeed", "speedAlongPath", "turnCorrection", "turnCorrectionP", "turnCorrectionI", "turnCorrectionD", "turnCorrectionF");
    double downCorrectionScale;
    private PID speedPID;
    private PID turnPID;
    private double maxPower;
    private double leftPower = .6;
    private double rightPower = .6;

    //Correction values
//    private static final double SPEED_SCALE = .001;
    private static final double TURN_SCALE = .01;
    double fCoeff;

    public HeadingController(Path path, double maxPower) {
        this.path = path;
        this.maxPower = maxPower;

        ParameterLookup lookup = FTCUtilities.getParameterLookup();

        double pCoeff = lookup.getParameter("p");
        double dCoeff = lookup.getParameter("d");
        fCoeff = lookup.getParameter("f");
        speedPID = new PID(.001, 0.0, .001, 5);
        turnPID = new PID(pCoeff, 0.0, dCoeff, 5);

        logger.startWriting();
    }

    public Powers getUpdatedPowers(Position robotPosition, Velocity robotVelocity) {
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        if (!targetLocation.pathFinished) {
            double targetSpeed = targetLocation.lookAheadSpeed;

            double speedAlongPath = (robotVelocity.dx * targetLocation.pathDeltaX) + (robotVelocity.dy * targetLocation.pathDeltaY);
            speedAlongPath /= targetLocation.pathSegmentLength;

            PID.Corrections speedCorrections = speedPID.getCorrection(speedAlongPath, targetSpeed);
            double totalSpeedCorrection = speedCorrections.totalCorrection;
            if (totalSpeedCorrection < 0) {
                totalSpeedCorrection *= DOWN_AMPLIFIER;
            }

            leftPower += totalSpeedCorrection;
            rightPower += totalSpeedCorrection;

            PID.Corrections turnCorrections = turnPID.getCorrection(targetLocation.distanceToRobot, 0);
            double lookAheadTurnCorrection = (targetLocation.lookAheadTurn * fCoeff);

            double totalTurnCorrection = turnCorrections.totalCorrection + lookAheadTurnCorrection;

            leftPower -= totalTurnCorrection;
            rightPower += totalTurnCorrection;

            logger.append("targetSpeed", String.valueOf(targetSpeed));
            logger.append("speedCorrection", String.valueOf(totalSpeedCorrection));
            logger.append("speedCorrectionP", String.valueOf(speedCorrections.correctionP));
            logger.append("speedCorrectionI", String.valueOf(speedCorrections.correctionI));
            logger.append("speedCorrectionD", String.valueOf(speedCorrections.correctionD));
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
            logger.append("turnCorrection", String.valueOf(totalTurnCorrection));
            logger.append("turnCorrectionP", String.valueOf(turnCorrections.correctionP));
            logger.append("turnCorrectionI", String.valueOf(turnCorrections.correctionI));
            logger.append("turnCorrectionD", String.valueOf(turnCorrections.correctionD));
            logger.append("turnCorrectionF", String.valueOf(lookAheadTurnCorrection));

            //Clip powers to maxPower by higher power
            double higherPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (higherPower > maxPower) {
                leftPower = (leftPower / higherPower) * maxPower;
                rightPower = (rightPower / higherPower) * maxPower;
            }

            if (leftPower < .2) {
                leftPower = .2;
            }
            if (rightPower < .2) {
                rightPower = .2;
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
