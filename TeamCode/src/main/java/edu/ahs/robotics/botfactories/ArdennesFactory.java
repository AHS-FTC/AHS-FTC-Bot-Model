package edu.ahs.robotics.botfactories;

import com.qualcomm.hardware.bosch.BNO055IMU;

import java.util.HashMap;
import java.util.Map;

import edu.ahs.robotics.hardware.ChassisMotors;
import edu.ahs.robotics.hardware.DriveUnit;
import edu.ahs.robotics.hardware.GearRatio;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.sensors.IMU;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.hardware.Robot;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;

public class ArdennesFactory extends BotFactory {

    @Override
    public Robot createRobot() {
        MecanumChassis mecanumChassis = makeChassis();
        Robot robot = new Robot(mecanumChassis);

        return robot;
    }

    private MecanumChassis makeChassis() {
        //Set Gear Ratio
        GearRatio driveGearRatio = new GearRatio(1,1);
        //Set Wheel Diameter in inches and Motor Type. These traits are shared by all chassis drive units
        DriveUnit.Config config = new DriveUnit.Config(driveGearRatio, 3.94, MotorHashService.MotorTypes.AM_20);

        //Make a HashMap that maps motors to their flip status. True indicates the motor runs reverse.
        Map<ChassisMotors.Mecanum, Boolean> driveFlips  = new HashMap<>();

        BNO055IMU bnoImu = FTCUtilities.getIMU("imu");
        IMU imu = new IMU(bnoImu);
        OdometrySystem odometrySystem = makeOdometrySystem(imu);

        driveFlips.put(ChassisMotors.Mecanum.FRONTLEFT, false); //todo check directions
        driveFlips.put(ChassisMotors.Mecanum.FRONTRIGHT, true);
        driveFlips.put(ChassisMotors.Mecanum.BACKLEFT, false);
        driveFlips.put(ChassisMotors.Mecanum.BACKRIGHT, true);

        return new MecanumChassis(config, driveFlips, imu);
    }

    private  OdometrySystem makeOdometrySystem(IMU imu){

        return new OdometrySystem(FTCUtilities.getMotor("intakeR"), FTCUtilities.getMotor("intakeL"), imu, 2.3622);
    }
}
