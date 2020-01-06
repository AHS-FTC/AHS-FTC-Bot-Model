package edu.ahs.robotics.hardware;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.control.HeadingController;
import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.pid.PositionPID;
import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Velocity;
import edu.ahs.robotics.control.pid.VelocityPID;
import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.control.Point;

public class MecanumChassis extends Chassis {

    private static final double ROBOT_WIDTH = 333;
    public static final int MIN_TARGET_DISTANCE = 5;
    public static final double DISTANCE_PER_360 = 43.8; //Tuned - real value 44.3 from getDistance xWheels //41.2 yWheels
    private static final double LEFT_INITIAL_SHIFT = 0;
    private static final double LEFT_INITIAL_SCALE = 1.02;
    private SingleDriveUnit frontLeft;
    private SingleDriveUnit frontRight;
    private SingleDriveUnit backLeft;
    private SingleDriveUnit backRight;

    // Motor shortcuts
    private ChassisMotors.Mecanum FRONT_LEFT = ChassisMotors.Mecanum.FRONTLEFT;
    private ChassisMotors.Mecanum FRONT_RIGHT = ChassisMotors.Mecanum.FRONTRIGHT;
    private ChassisMotors.Mecanum BACK_LEFT = ChassisMotors.Mecanum.BACKLEFT;
    private ChassisMotors.Mecanum BACK_RIGHT = ChassisMotors.Mecanum.BACKRIGHT;

    private OdometrySystem odometrySystem;

    private Odometer leftOdometer;
    private Odometer rightOdometer;


    public MecanumChassis(DriveUnit.Config driveUnitConfig, OdometrySystem odometrySystem) {
        super();
        frontLeft = new SingleDriveUnit(FRONT_LEFT.getDeviceName(), driveUnitConfig, false);
        frontRight = new SingleDriveUnit(FRONT_RIGHT.getDeviceName(), driveUnitConfig, true);
        backLeft = new SingleDriveUnit(BACK_LEFT.getDeviceName(), driveUnitConfig, false);
        backRight = new SingleDriveUnit(BACK_RIGHT.getDeviceName(), driveUnitConfig, true);

        this.odometrySystem = odometrySystem;
        leftOdometer = odometrySystem.getX1Odometer(); // this can easily change based on the definition of the x1 odometer
        rightOdometer = odometrySystem.getX2Odometer();
    }

//    public MecanumChassis(DriveUnit.Config driveUnitConfig, IMU imu, OdometrySystemImpl odometrySystem, String xMotorName, String yMotorName, double odometryWheelDiameter) {
//        this(driveUnitConfig, imu);
//
//        odometrySystem = new OdometrySystemImpl(FTCUtilities.getMotor(xMotorName), FTCUtilities.getMotor(yMotorName), imu, odometryWheelDiameter);
//    }


    /*private void motionInterpreter(PointTurn pointTurn){
        if(pointTurn.type == PointTurn.Type.RELATIVE){
            double startHeading = imu.getHeading();
            double deltaHeading = 0;
            double rampUp, rampDown;
            double power;
            double headingTo;
            double u = 0.10, d = 0.04;
            double minRampUp = .25;
            double minRampDown = .12;

            while (deltaHeading < pointTurn.targetHeading){
                deltaHeading = imu.getHeading() - startHeading;//done in the domain of delta heading because relative
                headingTo = pointTurn.targetHeading - deltaHeading;

                rampUp = Math.max(u*Math.sqrt(deltaHeading),minRampUp);
                rampDown = Math.max(d*Math.sqrt(headingTo),minRampDown);

                power = Math.min(rampUp, Math.min(rampDown, pointTurn.power));

                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);

                FTCUtilities.OpLogger("Delta Heading", imu.getHeading());
            }
            setPowerAll(0);
            //Logger.getInstance().stopWriting();

        } else if(pointTurn.type == PointTurn.Type.ABSOLUTE){

            /// not a thing yet --------------------------------
            double heading = imu.getHeading();
            int directionMultiplier = 1;

            if(pointTurn.targetHeading < heading) {//todo check for targetHeading directions, make sure this is right and not stupid
                directionMultiplier = -1;
            }
            while (directionMultiplier*heading < directionMultiplier*pointTurn.targetHeading){ //in theory, multiplying both sides by -1 effectively flips inequality. in theory
                heading = imu.getHeading();

                frontLeft.setPower(pointTurn.power);
                frontRight.setPower(-pointTurn.power);
                backLeft.setPower(pointTurn.power);
                backRight.setPower(-pointTurn.power);

                FTCUtilities.OpLogger("Heading", imu.getHeading());
            }
            setPowerAll(0);
        }
    }*/

    public void setPowerAll(double motorPower) {
        frontRight.setPower(motorPower);
        frontLeft.setPower(motorPower);
        backRight.setPower(motorPower);
        backLeft.setPower(motorPower);
    }

    public void stopMotors() {
        setPowerAll(0);
    }

    public void drive3Axis(double forward, double strafe, double turn) {

        double frontLeftPower = forward - strafe - turn;
        double frontRightPower = forward + strafe + turn;
        double backLeftPower = forward + strafe - turn;
        double backRightPower = forward - strafe + turn;


        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }

    public void arc(double angle, double radius, double maxPower, boolean rightTurn) {
        double minRampUp = .65;
        double minRampDown = .5;

        arc(angle, radius, maxPower, rightTurn, minRampUp, minRampDown, 5000);
    }


    public void arc(double angle, double radius, double maxPower, boolean rightTurn, double minRampUp, double minRampDown, long timeOut) {

        double leftTarget;
        double rightTarget;
        double innerTarget = angle * 2 * Math.PI * Math.abs(radius) / 360;
        double outerTarget = angle * 2 * Math.PI * (Math.abs(radius) + ROBOT_WIDTH) / 360;
        if (rightTurn) {
            leftTarget = outerTarget;
            rightTarget = innerTarget;
        } else {
            leftTarget = innerTarget;
            rightTarget = outerTarget;
        }
        FTCUtilities.sleep(1000);
        rawDrive(leftTarget, rightTarget, maxPower, minRampUp, minRampDown, .03, .01, timeOut);
    }

    public void driveStraight(double distance, double maxPower) {
        driveStraight(distance, maxPower, .55, .45, 5000);
    }
    
    public void driveStraight(double distance, double maxPower, double minRampUp, double minRampDown, long timeOut) {
        rawDrive(distance, distance, maxPower, minRampUp, minRampDown, .02, .005, timeOut);
    }

    public void pivot(double angle, double maxPower) {
        double minRampUp = .6;
        double minRampDown = .55;

        pivot(angle, maxPower, minRampUp, minRampDown, 3000);
    }

    public void pivot(double angle, double maxPower, double minRampUp, double minRampDown, long timeOut) {
        double leftTarget = (angle * DISTANCE_PER_360) / 360.0;
        double rightTarget = -1 * leftTarget;
        rawDrive(leftTarget, rightTarget, maxPower, minRampUp, minRampDown, .03, .06, timeOut);
    }


    private void rawDrive(double leftTarget, double rightTarget, double maxPower, double minRampUp, double minRampDown, double upScale, double downScale, long timeout) {

        if (maxPower < 0 || maxPower > 1.0) {
            throw new Warning("maxPower " + maxPower + " must be between 0 and 1");
        }
        if (Math.abs(leftTarget) < MIN_TARGET_DISTANCE || Math.abs(rightTarget) < MIN_TARGET_DISTANCE) {
            throw new Warning("Abs(targets) must be > MIN_TARGET_DISTANCE, " + MIN_TARGET_DISTANCE + " left target " + leftTarget + " right target " + rightTarget);
        }

        double maxTarget = Math.abs(leftTarget) > Math.abs(rightTarget) ? Math.abs(leftTarget) : Math.abs(rightTarget);

        final double correctionScale = 0.05; //Was 0.05

        leftOdometer.reset();
        rightOdometer.reset();

        try {
            double powerLeft = inversePower(((leftTarget / maxTarget) * (minRampUp) + LEFT_INITIAL_SHIFT) * LEFT_INITIAL_SCALE);
            double powerRight = inversePower((rightTarget / maxTarget) * (minRampUp));
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < timeout) {
                double leftDistance = leftOdometer.getDistance();
                double rightDistance = rightOdometer.getDistance();

                double leftDistanceRatio = leftDistance / leftTarget;
                double rightDistanceRatio = rightDistance / rightTarget;
                double averageDistanceRatio = (leftDistanceRatio + rightDistanceRatio) / 2;

                if (Math.abs(leftDistanceRatio) >= 1.0 && Math.abs(rightDistanceRatio) >= 1.0) { //was || before
                    break;
                }

                double maxDistance = Math.abs(leftDistance) > Math.abs(rightDistance) ? Math.abs(leftDistance) : Math.abs(rightDistance);
                double maxRemaining = maxTarget - maxDistance;
                double rampUp = (upScale * maxDistance) + minRampUp;
                double rampDown = (downScale * maxRemaining) + minRampDown; //distanceTo accounts for flip across y axis and x offset

                double targetPower = inversePower(Math.min(rampUp, Math.min(rampDown, maxPower)));

                double adjustLeft = correctionScale * (averageDistanceRatio - leftDistanceRatio);
                double adjustRight = correctionScale * (averageDistanceRatio - rightDistanceRatio);

                targetPower -= Math.max(adjustLeft, adjustRight);

                powerLeft += (adjustLeft * Math.signum(leftTarget));
                powerRight += (adjustRight * Math.signum(rightTarget));

                double higherPower = Math.max(Math.abs(powerLeft), Math.abs(powerRight));
                double rampRatio = targetPower / higherPower;

                powerLeft *= rampRatio;
                powerRight *= rampRatio;

                frontLeft.setPower(powerLeft);
                backLeft.setPower(powerLeft);
                frontRight.setPower(powerRight);
                backRight.setPower(powerRight);

                FTCUtilities.addData("power left", powerLeft);
                FTCUtilities.addData("power right", powerRight);
                FTCUtilities.addData("left ratio", leftDistanceRatio);
                FTCUtilities.addData("right ratio", rightDistanceRatio);
                FTCUtilities.addData("max remaining", maxRemaining);
                FTCUtilities.addData("leftDistance", leftDistance);
                FTCUtilities.addData("rightDistance", rightDistance);
                FTCUtilities.addData("ramp ratio", rampRatio);
                FTCUtilities.updateOpLogger();

            }
        } finally {
            setPowerAll(0);
        }
        //Logger.getInstance().stopWriting();
    }

    public void velocityDrive(Path path, double maxSpeed){
        OdometrySystem.State initialState = getState();

        Position initialPosition = initialState.position;
        /*Velocity initialVelocity = getVelocity();*/

        double distanceToEnd = path.getTargetLocation(initialPosition).distanceToEnd;

        double leftPower = 0.1, rightPower = 0.1; //initial power vals programmed in

        double rampDownScale = 3; //scales the sqrt graph for ramp down. Because all parabolas are similar, this is the only possible scale factor.

        VelocityPID.Config config = new VelocityPID.Config();
        config.setSpeedParams(0.01, 0.0005,  -0.001);
        config.setDirectionParams(0.01,0.0005,-0.001);

        VelocityPID pid = new VelocityPID(config);
        VelocityPID.Correction correction;

        if(!odometrySystem.isRunning()){
            throw new Warning("tried to do a velocityDrive without the odometrySystem running");
        }

        while (distanceToEnd > 0.05) { //0.05 is quite arbitrary
            double targetSpeed;

            OdometrySystem.State state = getState();

            Position currentPosition = state.position;
            Velocity currentVelocity = state.velocity;

            Path.Location location = path.getTargetLocation(currentPosition);

            Velocity targetVelocity = Velocity.makeVelocity(location.pathDeltaX,location.pathDeltaY);

            distanceToEnd = path.getTargetLocation(currentPosition).distanceToEnd; //this approaches zero, effectively 'flipping' ramp down along y axis

            targetSpeed = Math.min(maxSpeed, rampDownScale * Math.sqrt(distanceToEnd));

            // target speed math: https://www.desmos.com/calculator/31kvlxmlhn

            targetVelocity.scaleMagnitude(targetSpeed); //scale the velocity vector

            correction = pid.getCorrection(currentVelocity,targetVelocity); //get the corrections

            leftPower += correction.speed; //speed up all motors
            rightPower += correction.speed;

            leftPower -= correction.direction; // make direction adjustment
            rightPower += correction.direction;

            frontLeft.setPower(leftPower); //set powers
            backLeft.setPower(leftPower);

            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            //FTCUtilities.addData("leftPower", leftPower);
            //FTCUtilities.addData("rightPower", rightPower);
            //FTCUtilities.updateOpLogger();
        }
    }

    /**
     * Uses XYHeading PID to navigate/adjust position to a point on the field.
     * @param point A cartesian point on the field that the method will navigate to.
     * @param timeOut delta time in millis to end the main correction loop
     */
    public void goToPointWithPID(Point point, long timeOut){
        Position currentPosition = getState().position;
        Position targetPosition = new Position(point, 0);
        double frontLeftPower = 0, frontRightPower = 0;
        double backLeftPower = 0, backRightPower = 0;

        PositionPID.Correction correction;
        MecanumVectors mecanumVectorCorrections;

        PositionPID.Config pidConfig = new PositionPID.Config();
        pidConfig.setYParameters(0.01,0.001, -0.05);
        pidConfig.setXParameters(0.01,0.001, -0.05);
        pidConfig.setHeadingParameters(0,0,0);

        PositionPID pid = new PositionPID(pidConfig);

        long startTime = FTCUtilities.getCurrentTimeMillis();

        //odometrySystem.start(); // commented out because this should probably be handled at a higher level

        if(!odometrySystem.isRunning()){
            throw new Warning("tried to do a goToPointWithPID without the odometrySystem running");
        }

        while (FTCUtilities.getCurrentTimeMillis() - startTime < timeOut && currentPosition.distanceTo(targetPosition) > 0.1){
            currentPosition = getState().position;

            FTCUtilities.addData("x", currentPosition.x);
            FTCUtilities.addData("y", currentPosition.y);
            FTCUtilities.addData("heading", currentPosition.heading);
            FTCUtilities.updateOpLogger();

            correction = pid.getCorrection(currentPosition,targetPosition);

            //cast corrections to mecanum vectors
            mecanumVectorCorrections = MecanumVectors.convertLocalVectorsToMecanumVectors(correction.x, correction.y);

            frontLeftPower += mecanumVectorCorrections.forwardRight; // may change depending on wheel config
            backRightPower += mecanumVectorCorrections.forwardRight;

            frontRightPower += mecanumVectorCorrections.forwardLeft;
            backLeftPower += mecanumVectorCorrections.forwardLeft;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }
        //odometrySystem.stop();
    }

    /**
     * Sets the position of the robot using the odometrySystem.setPosition() method.
     * Should be utilized at the start at an OpMode to clarify your starting position.
     * <a href ="https://docs.google.com/drawings/d/1CasrlxBprQIvFcZTe8vHDQQWJD0nevVGOtdnTtcSpAw/edit?usp=sharing">Utilizes standard coordinate conventions</a>
     * @param x X position in inches
     * @param y Y position in inches
     * @param heading Heading in radians. Use the Math.toRadians() method if you have no guts
     */
    public void setPosition(double x, double y, double heading){
        odometrySystem.setPosition(x, y, heading);
    }

    private double inversePower(double power) {
        //   return power * Math.abs(power);
        return Math.signum(power) * Math.pow(power, 2);
    }

    public void followPath(Path path) {
        HeadingController headingController = new HeadingController(path,  1); //Max power before inversion
        HeadingController.Powers powers;
        do {
            OdometrySystem.State state = getState();
            powers = headingController.getUpdatedPowers(state);

            double inverseLeft = inversePower(powers.leftPower);
            double inverseRight = inversePower(powers.rightPower);

            frontLeft.setPower(inverseLeft);
            backLeft.setPower(inverseLeft);

            frontRight.setPower(inverseRight);
            backRight.setPower(inverseRight);
        }
        while (!powers.pathFinished);

    }

    /**
     * @return Global Position on the field
     */
    public OdometrySystem.State getState(){
        return odometrySystem.getState();
    }

    /**
     * Begins tracking position by echoing the OdometrySystemImpl.start() method.
     */
    public void startOdometrySystem(){
        odometrySystem.start();
    }

    /**
     * Stops odom tracking by echoing the OdometrySystemImpl.stop() method. Call this in your OpMode stop method.
     */
    public void stopOdometrySystem(){
        odometrySystem.stop();
    }

    /**
     * A class that contains vectors in local space along the axis of Mecanum Drive control.
     * Assumes a standard configuration with 45 degree wheel pitch.
     * Marked as package-private to enable unit testing
     * @author Alex Appleby
     */
     static class MecanumVectors{
         double forwardRight;
         double forwardLeft;

        MecanumVectors(double forwardRight, double forwardLeft) {
            this.forwardRight = forwardRight;
            this.forwardLeft = forwardLeft;
        }

        /**
         * Converts local XY vectors to vectors along the axes of mecanum wheel driving
         * @param x Magnitude of Local X vector
         * @param y Magnitude of Local Y vector
         */
         static MecanumVectors convertLocalVectorsToMecanumVectors(double x, double y){

            double forwardRight = (x - y) / 2;
            double forwardLeft = (x + y) / 2;

            return new MecanumVectors(forwardRight, forwardLeft);
        }
    }
}
