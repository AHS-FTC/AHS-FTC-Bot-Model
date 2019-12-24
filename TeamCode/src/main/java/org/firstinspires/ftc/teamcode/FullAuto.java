package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.hardware.sensors.ArdennesSkyStoneDetector;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;

public class FullAuto {

    private ElapsedTime runtime = new ElapsedTime();
    private Ardennes ardennes;
    private Intake intake;
    private MecanumChassis chassis;
    private Slides slides;
    private SerialServo foundationServoLeft;
    private SerialServo foundationServoRight;
    private SerialServo gripper;
    private SerialServo yslide;
    private SerialServo intakeServo;
    private SerialServo wrist;
    private TriggerDistanceSensor gripperTrigger;
    private TriggerDistanceSensor intakeTrigger;
    private ArdennesSkyStoneDetector detector;
    private boolean blueSide;
    private ArdennesSkyStoneDetector.SkyStoneConfigurations stoneConfiguration;

    public FullAuto(boolean blueSide) {
        this.blueSide = blueSide;
    }

    public void init() {

        MotorHashService.init();
        ardennes = new Ardennes();
        detector = new ArdennesSkyStoneDetector(false, blueSide);
        intake = ardennes.getIntake();
        chassis = ardennes.getChassis();
        slides = ardennes.getSlides();
        foundationServoLeft = ardennes.getLeftFoundation();
        foundationServoRight = ardennes.getRightFoundation();
        gripper = ardennes.getGripper();
        intakeServo = ardennes.getIntakeServo();
        yslide = ardennes.getySlide();
        wrist = ardennes.getWrist();
        gripperTrigger = ardennes.getGripperTrigger();
        intakeTrigger = ardennes.getIntakeTrigger();
        slides.resetEncoders();

        wrist.setPosition(-.5);
        gripper.setPosition(0);
        foundationServoLeft.setPosition(0);
        foundationServoRight.setPosition(0);
        yslide.setPosition(0);
        intakeServo.setPosition(0);
        FTCUtilities.addData("init", "finished");
        FTCUtilities.updateOpLogger();
    }

    public void afterStart() {
        intakeServo.setPosition(1);

        stoneConfiguration = detector.look();

        if (ArdennesSkyStoneDetector.SkyStoneConfigurations.ONE_FOUR == stoneConfiguration) {
            oneFourPlan();
        } else if (ArdennesSkyStoneDetector.SkyStoneConfigurations.TWO_FIVE == stoneConfiguration) {
            twoFivePlan();
        } else threeSixPlan();

        FTCUtilities.sleep(300);
        pivot(-85, .7);
        FTCUtilities.sleep(300);
        chassis.driveStraight(-200,.7, .65, .7,2000);
        foundationServoLeft.setPosition(1);
        foundationServoRight.setPosition(1);
        FTCUtilities.sleep(500);
        customArc(90,80,1,true, .7,.7,3000);
        chassis.driveStraight(-300,1,.7,.7,2000);
        FTCUtilities.sleep(300);
        //chassis.driveStraight(-200, 1, .7, .7);
        foundationServoLeft.setPosition(0);
        foundationServoRight.setPosition(0);
    }


    private void oneFourPlan() {
//        chassis.driveStraight(100, .75);
//        //pivot(-16, .93);
//        arc(15,1300, .65, false);
//        intake.startIntakeWaitForBlock(intakeTrigger);
//        //chassis.driveStraight(400, .93);
//        chassis.driveStraight(1000, .65);
//        chassis.driveStraight(-120,.8);
//        arc(-73,500,.93,true);
//        FTCUtilities.sleep(500);
//        chassis.driveStraight(-1100, .85);
//        FTCUtilities.sleep(500);
//        pivot(-70, .93);
//        FTCUtilities.sleep(200);
//        chassis.driveStraight(-250, .65);
//        foundationServoLeft.setPosition(1);
//        foundationServoRight.setPosition(1);
//        FTCUtilities.sleep(500);
//        //arc(130, 30,1,true);
//        pivot(90, .93);
//        chassis.driveStraight(-200, .93);
//        foundationServoLeft.setPosition(0);
//        foundationServoRight.setPosition(0);
//        FTCUtilities.sleep(300);
//        chassis.driveStraight(1500, .93);

        intake.startIntakeWaitForBlock(intakeTrigger);
        arc(25,2400, .7, false);
        arc(-67, 900, .8, true);
        chassis.driveStraight(-1000,.8);

//        arc(90, 300, .93, true);

        /*chassis.driveStraight(500, 1);
        chassis.pivot(-30, .4);
        intake.startIntakeWaitForBlock(ardennes.getIntakeTrigger());
        chassis.driveStraight(450, .3);
        */

    }

    private void twoFivePlan() {
        intake.startIntakeWaitForBlock(gripperTrigger);
        arc(5,13000, .7, false);
        arc(-73,600,.93, true);
        chassis.driveStraight(-1100, .93);



        /*pivot(10, .4);
        chassis.driveStraight(600, 1);
        pivot(-40, .4);
        intake.startIntakeWaitForBlock(ardennes.getIntakeTrigger());
        chassis.driveStraight(450, .3);
        chassis.driveStraight(-450, 1);
        pivot(-57, .4);
        chassis.driveStraight(-1700, .8);
        pivot(-87, .5);
        chassis.driveStraight(-200, .4);
        foundationServoLeft.setPosition(0);
        foundationServoRight.setPosition(1);
        FTCUtilities.sleep(1500);
        chassis.driveStraight(700, 1);
        pivot(93, .5);
        foundationServoLeft.setPosition(1);
        foundationServoRight.setPosition(1);
        FTCUtilities.sleep(700);
        */

    }

    private void threeSixPlan() {
        intake.startIntakeWaitForBlock(gripperTrigger);
        arc(13, 5000, .7, true);
        arc(-80, 500, .93, true);
        chassis.driveStraight(-1100, .93);

        /*chassis.pivot(-10, .4);
        chassis.driveStraight(550, 1);
        chassis.pivot(30, .5);
        intake.startIntakeWaitForBlock(ardennes.getIntakeTrigger());
        chassis.driveStraight(300, .3);
        */

    }

    private void pivot(double angle, double maxPower) {
        if (!blueSide) {
            chassis.pivot(-angle, maxPower);
        } else {
            chassis.pivot(angle, maxPower);
        }

    }

    private void customArc(double angle, double radius, double maxPower, boolean rightTurn, double minRampUp, double minRampDown, long timeOut) {
        if (!blueSide) {
            chassis.arc(angle, radius, maxPower, !rightTurn, minRampUp, minRampDown, timeOut);
        } else {
            chassis.arc(angle, radius, maxPower, rightTurn, minRampUp, minRampDown, timeOut);
        }
    }

    private void arc(double angle, double radius, double maxPower, boolean rightTurn) {
        if (!blueSide) {
            chassis.arc(angle, radius, maxPower, !rightTurn);
        } else {
            chassis.arc(angle, radius, maxPower, rightTurn);
        }
    }
}

