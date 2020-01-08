package edu.ahs.robotics.control;

import edu.ahs.robotics.control.pid.PID;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.util.ParameterLookup;

public class HeadingController {
    //Amplifies negative power corrections to deal with momentum while decelerating
    private static final double DOWN_AMPLIFIER = 1.5; // -- tuned --
    Path path;
    Logger logger = new Logger("TestAutoData");
    double downCorrectionScale;
    private PID speedPID;
    private PID positionPID;
    private PID directionPID;
    private PID curvaturePID;
    private double maxPower;
    private double leftPower = .2;
    private double rightPower = .2;

    //Correction values
    private static final double TURN_SCALE = .01;

    public HeadingController(Path path, double maxPower) {
        this.path = path;
        this.maxPower = maxPower;

        ParameterLookup lookup = FTCUtilities.getParameterLookup();

        double pPos = lookup.getParameter("p-pos");
        double dPos = lookup.getParameter("d-pos");

        double pDir = lookup.getParameter("p-dir");
        double dDir = lookup.getParameter("d-dir");

        double pArc = lookup.getParameter("p-arc");
        double dArc = lookup.getParameter("d-arc");


        speedPID = new PID(.004, 0.0, .006, 5); // -- tuned --
        positionPID = new PID(pPos, 0.0, dPos, 5);
        directionPID = new PID(pDir,0.0,dDir,5);
        curvaturePID = new PID(pArc,0.0,dArc,5);

        logger.startWriting();
    }

    public Powers getUpdatedPowers(OdometrySystem.State robotState) {
        Position robotPosition = robotState.position;
        Velocity robotVelocity = robotState.velocity;

        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        if (!targetLocation.pathFinished) {
            double targetSpeed = targetLocation.lookAheadSpeed;

            double speedAlongPath = (robotVelocity.dx * targetLocation.pathDeltaX) + (robotVelocity.dy * targetLocation.pathDeltaY);
            speedAlongPath /= targetLocation.pathSegmentLength;

            PID.Corrections speedCorrections = speedPID.getCorrection(targetSpeed - speedAlongPath);
            double totalSpeedCorrection = speedCorrections.totalCorrection;
            if (totalSpeedCorrection < 0) {
                totalSpeedCorrection *= DOWN_AMPLIFIER;
            }

            leftPower += totalSpeedCorrection;
            rightPower += totalSpeedCorrection;

            PID.Corrections positionCorrections = positionPID.getCorrection(targetLocation.distanceToRobot);

            double directionError = getDirectionError(targetLocation.pathDirection, robotPosition.heading);//for multi directional movements, velocity.direction may be more appropriate
            PID.Corrections directionCorrections = directionPID.getCorrection(directionError);

            double curvatureError = -1.0 * (targetLocation.lookAheadCurvature - robotState.travelCurvature); // curvature to the right is positive, thus the negative sign
            PID.Corrections curvatureCorrections = curvaturePID.getCorrection(curvatureError);

            double totalTurnCorrection = positionCorrections.totalCorrection + directionCorrections.totalCorrection + curvatureCorrections.totalCorrection;

            leftPower -= totalTurnCorrection;
            rightPower += totalTurnCorrection;

            logger.append("targetSpeed", String.valueOf(targetSpeed));
            logger.append("robotSpeed", String.valueOf(robotVelocity.speed()));
            //logger.append("speedCorrection", String.valueOf(totalSpeedCorrection));
            //logger.append("speedCorrectionP", String.valueOf(speedCorrections.correctionP));
            //logger.append("speedCorrectionI", String.valueOf(speedCorrections.correctionI));
            //logger.append("speedCorrectionD", String.valueOf(speedCorrections.correctionD));
            //logger.append("speedAlongPath", String.valueOf(speedAlongPath));
            //logger.append("distanceToRobot", String.valueOf(targetLocation.distanceToRobot));
            //logger.append("distanceToEnd", String.valueOf(targetLocation.distanceToEnd));
            logger.append("robotPositionX", String.valueOf(robotPosition.x));
            logger.append("robotPositionY", String.valueOf(robotPosition.y));
            logger.append("robotPositionHeading", String.valueOf(robotPosition.heading));
            logger.append("path Direction", String.valueOf(targetLocation.pathDirection));
            //logger.append("closestPointX", String.valueOf(targetLocation.closestPoint.x));
            //logger.append("closestPointY", String.valueOf(targetLocation.closestPoint.y));
            logger.append("direction error", String.valueOf(directionError));
            logger.append("direction correction", String.valueOf(directionCorrections.totalCorrection));
            logger.append("direction P correction", String.valueOf(directionCorrections.correctionP));
            logger.append("direction D correction", String.valueOf(directionCorrections.correctionD));

//            logger.append("turnCorrection", String.valueOf(totalTurnCorrection));
//            logger.append("turnCorrectionP", String.valueOf(positionCorrections.correctionP));
//            logger.append("turnCorrectionI", String.valueOf(positionCorrections.correctionI));
//            logger.append("turnCorrectionD", String.valueOf(positionCorrections.correctionD));
            logger.append("turnCorrectionF", String.valueOf(curvatureCorrections.totalCorrection));
            logger.append("lookAheadCurvature", String.valueOf(targetLocation.lookAheadCurvature));
            logger.append("robotCurvature", String.valueOf(robotState.travelCurvature));

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
