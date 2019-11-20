package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.Map;

import edu.ahs.robotics.autopaths.PointTurn;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.hardware.sensors.IMU;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.autopaths.PIDDController;
import edu.ahs.robotics.autopaths.PlanElement;
import edu.ahs.robotics.autopaths.functions.RampFunction;
import edu.ahs.robotics.autopaths.ArcMotion;
import edu.ahs.robotics.autopaths.ForwardMotion;
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

    public MecanumChassis(DriveUnit.Config driveUnitConfig, Map<ChassisMotors.Mecanum, Boolean> driveFlips) {
        super();
        frontLeft = new SingleDriveUnit(FRONT_LEFT.getDeviceName(),driveUnitConfig,driveFlips.get(FRONT_LEFT));
        frontRight = new SingleDriveUnit(FRONT_RIGHT.getDeviceName(), driveUnitConfig, driveFlips.get(FRONT_RIGHT));
        backLeft = new SingleDriveUnit(BACK_LEFT.getDeviceName(), driveUnitConfig, driveFlips.get(BACK_LEFT));
        backRight = new SingleDriveUnit(BACK_RIGHT.getDeviceName(), driveUnitConfig, driveFlips.get(BACK_RIGHT));
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
        } else {
            throw new Warning("Couldn't find a way to execute Planelement " + planElement.toString());
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


        //create Linear Function Object which specifies distance and speed.
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

            //calculate the error and total error...desired distance - actual distance
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
            //Logger.append(Logger.Cats.MOTORPOW,Double.toString(power));
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
            double heading = imu.getHeading();
//                while (){
//
//
//                }


        } else if(pointTurn.type == PointTurn.Type.ABSOLUTE){

        }
    }

    private void setPowerAll(double motorPower) {
        frontRight.setPower(motorPower);
        frontLeft.setPower(motorPower);
        backRight.setPower(motorPower);
        backLeft.setPower(motorPower);
    }
}
