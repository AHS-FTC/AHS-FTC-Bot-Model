package edu.ahs.robotics;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.HashMap;

public class MecanumChassis extends Chassis {

    private DriveUnit frontLeft;
    private DriveUnit frontRight;
    private DriveUnit backLeft;
    private DriveUnit backRight;

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

    public MecanumChassis(BotConfig botConfig) {
        super(botConfig);

        HashMap<MotorLocations, String> deviceNames = botConfig.getDriveMotorDeviceNames();
        //HashMap<MotorLocations, Boolean> deviceFlips = botConfig.getFlippedMotors();

        frontLeft = new DriveUnit(deviceNames.get(MotorLocations.FRONTLEFT), botConfig, botConfig.isFlipped(MotorLocations.FRONTLEFT));
        frontRight = new DriveUnit(deviceNames.get(MotorLocations.FRONTRIGHT), botConfig, botConfig.isFlipped(MotorLocations.FRONTRIGHT));
        backLeft = new DriveUnit(deviceNames.get(MotorLocations.BACKLEFT), botConfig, botConfig.isFlipped(MotorLocations.BACKLEFT));
        backRight = new DriveUnit(deviceNames.get(MotorLocations.BACKRIGHT), botConfig, botConfig.isFlipped(MotorLocations.BACKRIGHT));
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
