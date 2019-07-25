package edu.ahs.robotics;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.HashMap;

public class MecanumChassis extends Chassis {

    private SingleDriveUnit frontLeft;
    private SingleDriveUnit frontRight;
    private SingleDriveUnit backLeft;
    private SingleDriveUnit backRight;

    // Device names for drive units; corresponds to
    private static String FRONT_LEFT = "FL";
    private static String FRONT_RIGHT = "FR";
    private static String BACK_LEFT = "BL";
    private static String BACK_RIGHT = "BR";

    public MecanumChassis(DriveUnit.Config driveUnitConfig) {
        super();
        frontLeft = new SingleDriveUnit(FRONT_LEFT, driveUnitConfig, false);
        frontRight = new SingleDriveUnit(FRONT_RIGHT, driveUnitConfig, true);
        backLeft = new SingleDriveUnit(BACK_LEFT, driveUnitConfig, false);
        backRight = new SingleDriveUnit(BACK_RIGHT, driveUnitConfig, true);
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
        double desiredInchesTravelled = 0;
        double error = 0;
        double startTime = System.currentTimeMillis();
        double currentTime=0;
        double powerAdjustment = 0;
        double power = 0;

        //create Linear Function Object which specifies distance and speed.
        RampFunction forceFunction = new RampFunction(forwardMotion.travelDistance);

        //create PID object with specifed P,I,D coefficients
        PIDController myBasicPIDController = new PIDController(.1,0,0);

        while(actualInchesTravelled<forwardMotion.travelDistance){
            //get the current time
            double lastTime = currentTime;
            currentTime = System.currentTimeMillis()-startTime;
            double deltaTime = currentTime-lastTime;
            //calculate the error and total error...desired distance - actual distance
            desiredInchesTravelled = forceFunction.getDesiredDistance(currentTime);
            actualInchesTravelled = (frontRight.getDistance() + frontLeft.getDistance() + backRight.getDistance()+ backLeft.getDistance())/4; //This should be re-implemented for seperate feedback on every motor
            error = desiredInchesTravelled-1*actualInchesTravelled;

            //calculate the powerAdjustment based on the error

            powerAdjustment = myBasicPIDController.getPowerAdjustment(error,deltaTime);
            power += powerAdjustment;
            setPowerAll(power);
            //adjust the motor speed appropriately
            //adjustPowerAll(powerAdjustment);
            //pause for specified amount of time
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //FTCUtilities.OpSleep(1000);

            //Log Stuff!
/*
            FTCUtilities.OpLogger("Motor Power", power);
            FTCUtilities.OpLogger("Target Inches Travelled", desiredInchesTravelled);
            FTCUtilities.OpLogger("Inches Travelled", actualInchesTravelled);
            FTCUtilities.OpLogger("Power Adjustment", powerAdjustment);
            FTCUtilities.OpLogger("Error", error);
            FTCUtilities.OpLogger("Elapsed Time", currentTime);
            FTCUtilities.OpLogger("-------------", ":-----------");
*/

            myBasicPIDController.getMeanAndSD();

        }

        setPowerAll(0);
    }

    private void motionInterpreter(ArcMotion arcMotion) {

    }

    private void setPowerAll(double motorPower) {
        frontRight.setPower(motorPower);
        frontLeft.setPower(motorPower);
        backRight.setPower(motorPower);
        backLeft.setPower(motorPower);
    }
}
