package edu.ahs.robotics;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.HashMap;

public class MecanumChassis extends Chassis {

    private SingleDriveUnit frontLeft;
    private SingleDriveUnit frontRight;
    private SingleDriveUnit backLeft;
    private SingleDriveUnit backRight;

    public void execute(PlanElement planElement){
        if (planElement instanceof ForwardMotion) {
            if(planElement instanceof ForwardMotionBasic){
                motionInterpreter((ForwardMotionBasic)planElement);
            } else {
                motionInterpreter((ForwardMotion) planElement);
            }
        }
        else if (planElement instanceof ArcMotion){
            motionInterpreter((ArcMotion)planElement);
        }
        else {
            throw new Warning("Couldn't find a way to execute Planelement " + planElement.toString());
        }
    }

    public MecanumChassis(BotConfig botConfig) {
        super(botConfig);

        HashMap<MotorLocations, String> deviceNames = botConfig.getDriveMotorDeviceNames();
        //HashMap<MotorLocations, Boolean> deviceFlips = botConfig.getFlippedMotors();

        frontLeft = new SingleDriveUnit(deviceNames.get(MotorLocations.FRONTLEFT), botConfig, botConfig.isFlipped(MotorLocations.FRONTLEFT));
        frontRight = new SingleDriveUnit(deviceNames.get(MotorLocations.FRONTRIGHT), botConfig, botConfig.isFlipped(MotorLocations.FRONTRIGHT));
        backLeft = new SingleDriveUnit(deviceNames.get(MotorLocations.BACKLEFT), botConfig, botConfig.isFlipped(MotorLocations.BACKLEFT));
        backRight = new SingleDriveUnit(deviceNames.get(MotorLocations.BACKRIGHT), botConfig, botConfig.isFlipped(MotorLocations.BACKRIGHT));
    }
    public void turnOnMotor(){
        frontLeft.setPower(1);
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
        LinearFunction forceFunction = new LinearFunction(forwardMotion.travelDistance,12);

        //create PID object with specifed P,I,D coefficients
        PIDController myBasicPIDController = new PIDController(.1,0,0);

        while(actualInchesTravelled<forwardMotion.travelDistance){
            //get the current time
            currentTime = System.currentTimeMillis()-startTime;

            //calculate the error and total error...desired distance - actual distance
            desiredInchesTravelled = forceFunction.getDesiredDistance(currentTime);
            actualInchesTravelled = (frontRight.getDistance() + frontLeft.getDistance() + backRight.getDistance()+ backLeft.getDistance())/4; //This should be re-implemented for seperate feedback on every motor
            error = desiredInchesTravelled-actualInchesTravelled;

            //calculate the powerAdjustment based on the error

            powerAdjustment = myBasicPIDController.getPowerAdjustment(error);
            power += powerAdjustment;
            setPowerAll(power);
            //adjust the motor speed appropriately
            //adjustPowerAll(powerAdjustment);
            //pause for specified amount of time
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //FTCUtilities.OpSleep(1000);

            //Log Stuff!
            FTCUtilities.OpLogger("Motor Power", power);
            FTCUtilities.OpLogger("Inches Travelled", actualInchesTravelled);
            FTCUtilities.OpLogger("Power Adjustment", powerAdjustment);
            FTCUtilities.OpLogger("Error", error);
            FTCUtilities.OpLogger("Elapsed Time", currentTime);
            FTCUtilities.OpLogger("-------------", ":-----------");
            
        }

        setPowerAll(0);
    }

    private void motionInterpreter(ForwardMotionBasic forwardMotionBasic){
        double distance = forwardMotionBasic.travelDistance;
        double power = forwardMotionBasic.motorPower;
        double averageDistance = 0;

        //reset all encoders?

        setPowerAll(power);

        while (averageDistance < distance) {
            averageDistance = (backLeft.getDistance() + backRight.getDistance() + frontRight.getDistance() + frontLeft.getDistance())/4;
        }

        setPowerAll(0);
    }

    private void motionInterpreter(ArcMotion arcMotion){

    }

    private void setPowerAll(double motorPower) {
        frontRight.setPower(motorPower);
        frontLeft.setPower(motorPower);
        backRight.setPower(motorPower);
        backLeft.setPower(motorPower);
    }

    private void adjustPowerAll(double powerAdjustment) {
        frontRight.adjustPower(powerAdjustment);
        frontLeft.adjustPower(powerAdjustment);
        backRight.adjustPower(powerAdjustment);
        backLeft.adjustPower(powerAdjustment);
    }
}
