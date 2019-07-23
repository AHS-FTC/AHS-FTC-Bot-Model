package edu.ahs.robotics;


public class SummerBotConfig extends BotConfig {

    //Set Specs for

    private final double wheelDiameter = 3.94;
    private final Class chassisType = MecanumChassis.class;
    private final MotorHashService.MotorTypes driveMotorType = MotorHashService.MotorTypes.YJ_435;
    private final


    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public Class getChassisType() {
        return chassisType;
    }

    public MotorHashService.MotorTypes getDriveMotorType() {
        return driveMotorType;
    }

    public GearRatio getGearRatio() {
        return null;
    }

}
