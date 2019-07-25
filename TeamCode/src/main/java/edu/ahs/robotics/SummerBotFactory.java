package edu.ahs.robotics;

import java.util.HashMap;
import java.util.Map;

public class SummerBotFactory extends BotFactory {

    @Override
    public Robot createRobot() {
        GearRatio driveGearRatio = new GearRatio(1,2);
        DriveUnit.Config config = new DriveUnit.Config(driveGearRatio, 3.94, MotorHashService.MotorTypes.YJ_435);
        Map<ChassisMotors.Mecanum, Boolean> driveFlips  = new HashMap<>();

        driveFlips.put(ChassisMotors.Mecanum.FRONTLEFT, false);
        driveFlips.put(ChassisMotors.Mecanum.FRONTRIGHT, true);
        driveFlips.put(ChassisMotors.Mecanum.BACKLEFT, false);
        driveFlips.put(ChassisMotors.Mecanum.BACKRIGHT, true);

        MecanumChassis mecanumChassis = new MecanumChassis(config, driveFlips);
        Robot robot = new Robot(mecanumChassis);
        return robot;
    }
}
