package edu.ahs.robotics;

import java.util.HashMap;
import java.util.Map;

public class SummerBotFactory extends BotFactory {

    @Override
    public Robot createRobot() {
        GearRatio driveGearRatio = new GearRatio(1,2);
        DriveUnit.Config config = new DriveUnit.Config(driveGearRatio, 3.94, MotorHashService.MotorTypes.YJ_435);
        Map<ChassisMotors, Boolean> driveFlips  = new HashMap<>();

        driveFlips.put(ChassisMotors.FRONTLEFT, false);
        driveFlips.put(ChassisMotors.FRONTRIGHT, true);
        driveFlips.put(ChassisMotors.BACKLEFT, false);
        driveFlips.put(ChassisMotors.BACKRIGHT, true);

        MecanumChassis mecanumChassis = new MecanumChassis(config, driveFlips);
        Robot robot = new Robot(mecanumChassis);
        return robot;
    }
}
