package edu.ahs.robotics;

import android.view.SubMenu;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.HashMap;
import java.util.Map;

public class BotFactory {
    private static Class chassisType;
    private static MotorHashService.MotorTypes driveMotorType;
    private static double wheelDiameter;
    private static GearRatio driveGearRatio = new GearRatio();
    //private int[] driveGears = new int[]{1, 1}; //input, output
    private static boolean wasDriveGearRatioInitialized = false;
    private static HashMap<MotorLocations, String> driveMotorDeviceNames;
    private static HashMap<MotorLocations, Boolean> flippedMotors;
    private static boolean override = false;

    SummerBotConfig summerBotConfig = new SummerBotConfig();


    //set BotFactory Parameters prior to creation of main Robot object



    BotFactory.setDriveMotors(MotorHashService.MotorTypes.YJ_435);
    BotFactory.setDriveGears(1,2);

    deviceNames.put(MotorLocations.FRONTLEFT, "FL");
    deviceNames.put(MotorLocations.FRONTRIGHT, "FR");
    deviceNames.put(MotorLocations.BACKLEFT, "BL");
    deviceNames.put(MotorLocations.BACKRIGHT, "BR");

    motorFlips.put(MotorLocations.FRONTLEFT, false);
    motorFlips.put(MotorLocations.FRONTRIGHT, true);
    motorFlips.put(MotorLocations.BACKLEFT, false);
    motorFlips.put(MotorLocations.BACKRIGHT, true);

    summerBotFactory.setDriveMotorDeviceNames(deviceNames);
    summerBotFactory.setFlippedMotors(motorFlips);

    //Create main Robot Object
    Robot summerBot = new Robot();


    public void setChassisType(Class chassisType) {
        if ((chassisType.getSuperclass() != Chassis.class) && (chassisType != Chassis.class)) {
            throw new Error("The Chassis you specified is not a subclass of the Chassis Class");
        }
        this.chassisType = chassisType;
    }

    public void setDriveMotors(MotorHashService.MotorTypes driveMotorType) {
        this.driveMotorType = driveMotorType;
    }

    public void setWheelDiameter(double wheelDiameter) {//in inches (for now)
        if (wheelDiameter <= 0.0) {
            throw new Error("Wheel Diameter should be greater than zero");
        }
        this.wheelDiameter = wheelDiameter;
    }

    public void setDriveGears(int inputTeeth, int outputTeeth) {
        if (inputTeeth < 1 || outputTeeth < 1) {
            throw new Error("Your gearRatio teeth must be a positive nonzero integer");
        }
        driveGearRatio = new GearRatio(inputTeeth, outputTeeth);
        wasDriveGearRatioInitialized = true;
    }

    public void setDriveMotorDeviceNames(HashMap<MotorLocations, String> driveMotorDeviceNames) {
        this.driveMotorDeviceNames = driveMotorDeviceNames;
    }
    public void setFlippedMotors(HashMap flippedMotors){
        this.flippedMotors = flippedMotors;
    }

    public Class getChassisType() {
        return chassisType;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public MotorHashService.MotorTypes getDriveMotorType() {
        return driveMotorType;
    }

    public GearRatio getDriveGearRatio() {
        return driveGearRatio;
    }

    public HashMap<MotorLocations, String> getDriveMotorDeviceNames() {
        return driveMotorDeviceNames;
    }
    public HashMap<MotorLocations, Boolean> getFlippedMotors(){
        return flippedMotors;
    }
    public boolean isFlipped(MotorLocations motorLocation){
        return flippedMotors.get(motorLocation);
    }

    public void overrideConfigCheck(){
        override = true;
        FTCUtilities.OpLogger("Config Override", override);
    }

    public boolean isFullyConfigured() {
        boolean chassis, driveMotors, wheels, motorFlips; // are these things configured?
        chassis = chassisType != null;
        driveMotors = driveMotorType != null;
        wheels = wheelDiameter != 0;
        motorFlips = flippedMotors != null;

        if (!wasDriveGearRatioInitialized) {
            throw new Warning("The Drive Gear Ratio was never configured using BotFactory. DriveUnits will assume a 1:1 ratio.");
        }
        boolean configStatus = chassis && driveMotors && wheels && motorFlips;
        return configStatus || override;
    }

}


