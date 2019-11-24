package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.Map;

import edu.ahs.robotics.autocommands.autopaths.OdometryForwardMotion;
import edu.ahs.robotics.autocommands.autopaths.OdometryPointTurn;
import edu.ahs.robotics.autocommands.autopaths.PointTurn;
import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.hardware.sensors.IMU;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.autocommands.autopaths.PIDDController;
import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.autocommands.autopaths.functions.RampFunction;
import edu.ahs.robotics.autocommands.autopaths.ArcMotion;
import edu.ahs.robotics.autocommands.autopaths.ForwardMotion;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;

public class MecanumChassis extends Chassis {

    private SingleDriveUnit frontLeft;
    private SingleDriveUnit frontRight;
    private SingleDriveUnit backLeft;
    private SingleDriveUnit backRight;

    // Motor shortcuts
    private ChassisMotors.Mecanum FRONT_LEFT = ChassisMotors.Mecanum.FRONTLEFT;
    private ChassisMotors.Mecanum FRONT_RIGHT = ChassisMotors.Mecanum.FRONTRIGHT;
    private ChassisMotors.Mecanum BACK_LEFT = ChassisMotors.Mecanum.BACKLEFT;
    private ChassisMotors.Mecanum BACK_RIGHT = ChassisMotors.Mecanum.BACKRIGHT;

    private IMU imu;
    private OdometrySystem odometrySystem;

    private Odometer leftOdometer;
    private Odometer rightOdometer;


    public MecanumChassis(DriveUnit.Config driveUnitConfig, Map<ChassisMotors.Mecanum, Boolean> driveFlips) {
        super();
        frontLeft = new SingleDriveUnit(FRONT_LEFT.getDeviceName(),driveUnitConfig,driveFlips.get(FRONT_LEFT));
        frontRight = new SingleDriveUnit(FRONT_RIGHT.getDeviceName(), driveUnitConfig, driveFlips.get(FRONT_RIGHT));
        backLeft = new SingleDriveUnit(BACK_LEFT.getDeviceName(), driveUnitConfig, driveFlips.get(BACK_LEFT));
        backRight = new SingleDriveUnit(BACK_RIGHT.getDeviceName(), driveUnitConfig, driveFlips.get(BACK_RIGHT));

        leftOdometer = new Odometer("intakeL", 60.2967 , false);
        rightOdometer = new Odometer("intakeR", 60.79, true);
    }


    public MecanumChassis(DriveUnit.Config driveUnitConfig, Map<ChassisMotors.Mecanum, Boolean> driveFlips, IMU imu) {
        this(driveUnitConfig, driveFlips);
        this.imu = imu;
    }

    public MecanumChassis(DriveUnit.Config driveUnitConfig, Map<ChassisMotors.Mecanum, Boolean> driveFlips, IMU imu, OdometrySystem odometrySystem, String xMotorName, String yMotorName, double odometryWheelDiameter) {
        this(driveUnitConfig, driveFlips, imu);

        odometrySystem = new OdometrySystem(FTCUtilities.getMotor(xMotorName), FTCUtilities.getMotor(yMotorName), imu, odometryWheelDiameter);
    }


    public void execute(PlanElement planElement) {
        if (planElement instanceof ForwardMotion) {
            motionInterpreter((ForwardMotion) planElement);
        } else if (planElement instanceof ArcMotion) {
            motionInterpreter((ArcMotion) planElement);
        } else if(planElement instanceof PointTurn) {
            motionInterpreter((PointTurn) planElement);
        } else if (planElement instanceof OdometryForwardMotion){
            motionInterpreter((OdometryForwardMotion) planElement);
        } else if (planElement instanceof OdometryPointTurn){
            motionInterpreter((OdometryPointTurn) planElement);
        } else {
            throw new Warning("Couldn't find a way to executePlan PlanElement " + planElement.toString());
        }
    }

    private void motionInterpreter(ForwardMotion forwardMotion) {

        double actualInchesTravelled = 0;

        double startTime = System.currentTimeMillis();
        double currentTime=0;
        double power = 0;
        double lastROC =0;
        int loopCounter=0;
        int loopInterval = 50;


        //create Linear Function Object which specifies targetDistance and speed.
        RampFunction forceFunction = new RampFunction(forwardMotion.travelDistance);

        //create PID object with specifed P,I,D,DD coefficients
        PIDDController myBasicPIDDController = new PIDDController(0.01,.001,.00,0.26);

        while(actualInchesTravelled<forwardMotion.travelDistance && forwardMotion.timeOut>currentTime){
            //get the current time
            double lastTime = currentTime;
            currentTime = System.currentTimeMillis()-startTime;
            double deltaTime = currentTime-lastTime;
            //ensure a regular Sample Interval
            double sleepTime = Range.clip(loopInterval*loopCounter-currentTime,0,loopInterval);
            loopCounter+=1;

            try {
                Thread.sleep((int)sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            //calculate the error and total error...desired targetDistance - actual targetDistance
            double desiredInchesTravelled = forceFunction.getDesiredDistance(currentTime);
            actualInchesTravelled = (frontRight.getDistance() + frontLeft.getDistance() + backRight.getDistance()+ backLeft.getDistance())/4; //This should be re-implemented for seperate feedback on every motor
            double error = desiredInchesTravelled-actualInchesTravelled;
            double currentROC = (forceFunction.getDesiredDistance(currentTime+loopInterval)-forceFunction.getDesiredDistance(currentTime));
            //calculate the powerAdjustment based on the error
            double secondDerivative=(currentROC-lastROC);
            lastROC = currentROC;

            double powerAdjustment = myBasicPIDDController.getPowerAdjustment(error,deltaTime,secondDerivative);
            power += powerAdjustment;
            power = Range.clip(power,-1,1);
            setPowerAll(power);
            //adjust the motor speed appropriately
            //adjustPowerAll(powerAdjustment);
            //pause for specified amount of time

            //FTCUtilities.OpSleep(1000);

            //Log Stuff!
            Logger.append(Logger.Cats.ENCODERDIST,Double.toString(actualInchesTravelled));
            Logger.append(Logger.Cats.DESIDIST,Double.toString(desiredInchesTravelled));
            Logger.append(Logger.Cats.ERROR,Double.toString(error));
            //Logger.append(Logger.Cats.MOTORPOW,Double.toString(maxPower));
            //Logger.append(Logger.Cats.POWADJ,Double.toString(powerAdjustment));
            //Logger.append(Logger.Cats.TIME,Double.toString(currentTime));
            //FTCUtilities.OpLogger("Target Inches Travelled", desiredInchesTravelled);
            FTCUtilities.OpLogger("Inches Travelled", actualInchesTravelled);
           // FTCUtilities.OpLogger("Error", error);
            //FTCUtilities.OpLogger("Power Adjustment", powerAdjustment);
            FTCUtilities.OpLogger("Motor Power", power);
            //FTCUtilities.OpLogger("Elapsed Time", currentTime);
            //FTCUtilities.OpLogger("Second Derivative", secondDerivative);
            //myBasicPIDDController.getMeanAndSD();
            //FTCUtilities.OpLogger("-------------", ":-----------");
        }
        myBasicPIDDController.getMeanAndSD();

        setPowerAll(0);
    }

    private void motionInterpreter(ArcMotion arcMotion) {

    }

    private void motionInterpreter(PointTurn pointTurn){
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
            //Logger.getInstance().writeToFile();

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
    }

    private void motionInterpreter(OdometryForwardMotion motion){
        double distance = 0;
        double rampUp, rampDown;
        double distanceTo;
        double power, scaledPower;
        double minRampUp = .2;
        double minRampDown = .12;

        double u = 0.05, d = 0.013; // reciprocal of millimeters after which you will be at maxPower 1

        leftOdometer.reset();
        rightOdometer.reset();

        while(Math.abs(distance) < Math.abs(motion.targetDistance)){
            distance = Math.abs(leftOdometer.getDistance() + rightOdometer.getDistance())/2;
            distanceTo = Math.abs(motion.targetDistance) - Math.abs(distance);

            rampUp = Math.max(u*Math.sqrt(distance),minRampUp);
            rampDown = Math.max(d*Math.sqrt(distanceTo),minRampDown); //distanceTo accounts for flip across y axis and x offset

            power = Math.min(rampUp, Math.min(rampDown, motion.maxPower));

            FTCUtilities.addData("maxPower", power);
            FTCUtilities.addData("left", leftOdometer.getDistance());
            FTCUtilities.addData("right", rightOdometer.getDistance());
            FTCUtilities.updateOpLogger();

            setPowerAll(power);
        }
        setPowerAll(0);
    }

    private void motionInterpreter(OdometryPointTurn turn){
        double leftDistance, rightDistance, averageDistance = 0;
        double rampUp, rampDown;
        double power, scaledPower;
        double minRampUp = .2;
        double minRampDown = .2;
        double leftTarget = 243, rightTarget = -243; //250,-250 for almost perfect 90 deg
        double leftRemaining, rightRemaining;
        double averageRemaining = (Math.abs(leftTarget) + Math.abs(rightTarget))/2;
        double errorLeft, errorRight;
        double correctionScale = 0.01;

        double u = 0.07, d = 0.01; // reciprocal of millimeters after which you will be at maxPower 1

        leftOdometer.reset();
        rightOdometer.reset();

        while(averageRemaining > 0){
            leftDistance = leftOdometer.getDistance();
            rightDistance = rightOdometer.getDistance();

            averageDistance = (Math.abs(leftDistance) + Math.abs(rightDistance))/2;

            leftRemaining = Math.signum(leftTarget)*(leftTarget - leftDistance);
            rightRemaining = Math.signum(rightTarget)*(rightTarget - rightDistance);

            averageRemaining = (leftRemaining + rightRemaining)/2;

            rampUp = Math.max(u*Math.sqrt(averageDistance),minRampUp);
            rampDown = Math.max(d*Math.sqrt(averageRemaining),minRampDown); //distanceTo accounts for flip across y axis and x offset

            power = Math.min(rampUp, Math.min(rampDown, turn.maxPower));

            FTCUtilities.addData("power", power);
            FTCUtilities.addData("left r", leftRemaining);
            FTCUtilities.addData("right r", rightRemaining);
            FTCUtilities.addData("average remaining", averageRemaining);
            FTCUtilities.updateOpLogger();

            errorLeft = averageDistance - Math.abs(leftDistance);
            errorRight = averageDistance - Math.abs(rightDistance);


            double powerLeft = Math.signum(leftTarget) * (power + correctionScale * errorLeft);
            frontLeft.setPower(powerLeft);
            backLeft.setPower(powerLeft);

            double powerRight = Math.signum(rightTarget) * (power + correctionScale * errorRight);
            frontRight.setPower(powerRight);
            backRight.setPower(powerRight);

            Logger.append(Logger.Cats.DADJUSTMENT, String.valueOf(errorLeft));
            Logger.append(Logger.Cats.DDADJUSTMENT, String.valueOf(errorRight));
            Logger.append(Logger.Cats.ERROR, String.valueOf(powerLeft));
            Logger.append(Logger.Cats.ENCODERDIST, String.valueOf(powerRight));
            Logger.append(Logger.Cats.IADJUSTMENT, String.valueOf(leftDistance));
            Logger.append(Logger.Cats.DESIDIST, String.valueOf(rightDistance));

        }
        setPowerAll(0);
        Logger.getInstance().writeToFile();
    }

    private void setPowerAll(double motorPower) {
        frontRight.setPower(motorPower);
        frontLeft.setPower(motorPower);
        backRight.setPower(motorPower);
        backLeft.setPower(motorPower);
    }
}
