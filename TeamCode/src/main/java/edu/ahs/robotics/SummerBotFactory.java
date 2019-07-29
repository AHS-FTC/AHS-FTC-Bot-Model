package edu.ahs.robotics;

import java.util.HashMap;
import java.util.Map;

public class SummerBotFactory extends BotFactory {

    @Override
    public Robot createRobot() {
        MecanumChassis mecanumChassis = makeChassis();
        Robot robot = new Robot(mecanumChassis);
        return robot;
    }

    private MecanumChassis makeChassis() {
        //Set Gear Ratio
        GearRatio driveGearRatio = new GearRatio(1,2);
        //Set Wheel Diameter in inches and Motor Type. These traits are shared by all chassis drive units
        DriveUnit.Config config = new DriveUnit.Config(driveGearRatio, 3.94, MotorHashService.MotorTypes.YJ_435);

        //Make a HashMap that maps motors to their flip status. True indicates the motor runs reverse.
        Map<ChassisMotors.Mecanum, Boolean> driveFlips  = new HashMap<>();

        driveFlips.put(ChassisMotors.Mecanum.FRONTLEFT, false);
        driveFlips.put(ChassisMotors.Mecanum.FRONTRIGHT, true);
        driveFlips.put(ChassisMotors.Mecanum.BACKLEFT, false);
        driveFlips.put(ChassisMotors.Mecanum.BACKRIGHT, true);

        return new MecanumChassis(config, driveFlips);
    }
}
