package edu.ahs.robotics;

public class SummerBotFactory extends BotFactory {

    @Override
    public Robot createRobot() {
        GearRatio driveGearRatio = new GearRatio(1,2);
        DriveUnit.Config config = new DriveUnit.Config(driveGearRatio, 3.94, MotorHashService.MotorTypes.YJ_435);
        MecanumChassis mecanumChassis = new MecanumChassis(config);
        Robot robot = new Robot(mecanumChassis);
        return robot;
    }
}
