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
            motionInterpreter((ForwardMotion)planElement);
        }
        else if (planElement instanceof ArcMotion){
            motionInterpreter((ArcMotion)planElement);
        }
        else {
            throw new Warning("Couldn't find a way to execute Planelement " + planElement.toString());
        }
    }

    public MecanumChassis(BotFactory botFactory) {
        super(botFactory);

        HashMap<MotorLocations, String> deviceNames = botFactory.getDriveMotorDeviceNames();
        //HashMap<MotorLocations, Boolean> deviceFlips = botFactory.getFlippedMotors();

        frontLeft = new SingleDriveUnit(deviceNames.get(MotorLocations.FRONTLEFT), botFactory, botFactory.isFlipped(MotorLocations.FRONTLEFT));
        frontRight = new SingleDriveUnit(deviceNames.get(MotorLocations.FRONTRIGHT), botFactory, botFactory.isFlipped(MotorLocations.FRONTRIGHT));
        backLeft = new SingleDriveUnit(deviceNames.get(MotorLocations.BACKLEFT), botFactory, botFactory.isFlipped(MotorLocations.BACKLEFT));
        backRight = new SingleDriveUnit(deviceNames.get(MotorLocations.BACKRIGHT), botFactory, botFactory.isFlipped(MotorLocations.BACKRIGHT));
    }
    public void turnOnMotor(){
        frontLeft.setPower(1);
    }


    private void motionInterpreter(ForwardMotion forwardMotion) {

        //frontLeft.zeroDistance();
        //frontRight.zeroDistance();
        //backLeft.zeroDistance();
        //backRight.zeroDistance();

        double encoderAverage = 0;


        while(encoderAverage<forwardMotion.travelDistance){
            encoderAverage = (frontRight.getDistance() + frontLeft.getDistance() + backRight.getDistance()+ backLeft.getDistance())/4;
            FTCUtilities.OpLogger("EncoderAverage", encoderAverage);
            setPowerAll(forwardMotion.motorPower);
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
}
