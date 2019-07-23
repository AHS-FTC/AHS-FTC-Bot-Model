package edu.ahs.robotics;

import java.util.HashMap;

public abstract class BotConfig {

    public abstract double getWheelDiameter();
    public abstract Class getChassisType();
    public abstract MotorHashService.MotorTypes getDriveMotorType();
    public abstract GearRatio getGearRatio();

}

