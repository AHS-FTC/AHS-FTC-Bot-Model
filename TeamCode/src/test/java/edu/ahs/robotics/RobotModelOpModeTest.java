package edu.ahs.robotics;

import org.junit.Test;

import edu.ahs.robotics.hardware.Robot;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.util.MotorHashService;

public class RobotModelOpModeTest {
    private MotorHashService.MotorTypes driveMotorType = MotorHashService.MotorTypes.YJ_223;

    @Test
    public void testThis() {
        // Set up mock motors and device names
        FTCUtilities.startTestMode();
        DcMotorMock frontLeft = new DcMotorMock(driveMotorType);
        DcMotorMock frontRight = new DcMotorMock(driveMotorType);
        DcMotorMock backLeft = new DcMotorMock(driveMotorType);
        DcMotorMock backRight = new DcMotorMock(driveMotorType);

        FTCUtilities.addTestMotor(frontLeft, "FL");
        FTCUtilities.addTestMotor(frontRight, "FR");
        FTCUtilities.addTestMotor(backLeft, "BL");
        FTCUtilities.addTestMotor(backRight, "BR");

        RobotModelOpMode opMode = new RobotModelOpMode();
        Robot robot = opMode.initRobot();
        robot.execute();
        Logger.getInstance().writeToFile();

    }


}
