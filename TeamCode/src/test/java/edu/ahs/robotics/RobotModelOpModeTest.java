package edu.ahs.robotics;

import org.junit.Test;

public class RobotModelOpModeTest {
    private MotorHashService.MotorTypes driveMotorType = MotorHashService.MotorTypes.YJ_223;

    @Test
    public void testThis() {
        // That was a fantastic comment
        // Set up mock motors
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

    }


}
