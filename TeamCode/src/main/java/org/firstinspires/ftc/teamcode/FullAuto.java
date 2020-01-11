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
    private SerialServo capstone;
    private TriggerDistanceSensor gripperTrigger;
    //private TriggerDistanceSensor intakeTrigger;
    private ArdennesSkyStoneDetector detector;
    private boolean redSide;
    private ArdennesSkyStoneDetector.SkyStoneConfigurations stoneConfiguration;

    public FullAuto(boolean redSide) {
        this.redSide = redSide;
    }

    public void init() {

        MotorHashService.init();
        ardennes = new Ardennes();
        detector = new ArdennesSkyStoneDetector(false, redSide);
        intake = ardennes.getIntake();
        chassis = ardennes.getChassis();
        slides = ardennes.getSlides();
        foundationServoLeft = ardennes.getLeftFoundation();
        foundationServoRight = ardennes.getRightFoundation();
        gripper = ardennes.getGripper();
        yslide = ardennes.getySlide();
        capstone = ardennes.getCapstone();
        gripperTrigger = ardennes.getGripperTrigger();
        //intakeTrigger = ardennes.getIntakeTrigger();
        slides.resetEncoders();

        //capstone.setPosition(-.5);
        gripper.setPosition(0);
        foundationServoLeft.setPosition(0);
        foundationServoRight.setPosition(0);
        yslide.setPosition(0);
        FTCUtilities.addData("init", "finished");
        FTCUtilities.updateOpLogger();
    }

    public void afterStart() {

        stoneConfiguration = detector.look();

        if (!redSide) {
            if (ArdennesSkyStoneDetector.SkyStoneConfigurations.ONE_FOUR == stoneConfiguration) {
                oneFourPlanBlue();
            } else if (ArdennesSkyStoneDetector.SkyStoneConfigurations.TWO_FIVE == stoneConfiguration) {
                twoFivePlanBlue();
            } else threeSixPlanBlue();

            FTCUtilities.sleep(300);
            chassis.pivot(85, .4);
            FTCUtilities.sleep(300);
            chassis.driveStraight(-7.5,.3, .2, .2,2000); //200mm
            foundationServoLeft.setPosition(1);
            foundationServoRight.setPosition(1);
            FTCUtilities.sleep(500);
            chassis.driveStraight(14,1,.4,.4, 2000);
            chassis.arc(80,3.15,1,false, .4,.4,2500); //80mm
            chassis.driveStraight(-11.8,1,.4,.4,2000); //300 mm

        } else {
            if (ArdennesSkyStoneDetector.SkyStoneConfigurations.ONE_FOUR == stoneConfiguration) {
                oneFourPlanRed();
            } else if (ArdennesSkyStoneDetector.SkyStoneConfigurations.TWO_FIVE == stoneConfiguration) {
                twoFivePlanRed();
            } else threeSixPlanRed();

            FTCUtilities.sleep(300);
            chassis.pivot(-85, .4);
            FTCUtilities.sleep(300);
            chassis.driveStraight(-7.5,.3, .2, .2,2000); //200mm
            foundationServoLeft.setPosition(1);
            foundationServoRight.setPosition(1);
            FTCUtilities.sleep(500);
            chassis.driveStraight(14,1,.4,.4, 2000);
            chassis.arc(80,3.15,1,true, .4,.4,2500); //80mm
            chassis.driveStraight(-11.8,1,.4,.4,2000); //300 mm

        }


        FTCUtilities.sleep(300);
        foundationServoLeft.setPosition(0);
        foundationServoRight.setPosition(0);
        FTCUtilities.sleep(500);
        gripper.setPosition(1);
        FTCUtilities.sleep(1000);
        slides.setTargetLevel(2);
        slides.runToLevel();
        FTCUtilities.sleep(300);
        yslide.setPosition(1);
        FTCUtilities.sleep(1500);
        gripper.setPosition(0);
        FTCUtilities.sleep(1000);
        yslide.setPosition(0);
        FTCUtilities.sleep(1000);
        chassis.stopLogger();

    }


    private void oneFourPlanBlue() {

        //intake.startIntakeWaitForBlock(intakeTrigger);
        intake.startIntakeWaitForBlock(gripperTrigger);
        chassis.arc(30,72, .3, true); //2400
        chassis.arc(-63, 17, .8, false); //900
        chassis.driveStraight(-53,.8); //1000

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



//        arc(90, 300, .93, true);

        /*chassis.driveStraight(500, 1);
        chassis.pivot(-30, .4);
        intake.startIntakeWaitForBlock(ardennes.getIntakeTrigger());
        chassis.driveStraight(450, .3);
        */

    }

    private void twoFivePlanBlue() {
        //intake.startIntakeWaitForBlock(gripperTrigger);
        intake.startIntakeWaitForBlock(gripperTrigger);
        chassis.arc(6,450, .3, true); //13000
        chassis.arc(-81,15,.8, false); //600
        chassis.driveStraight(-50, .8); //1100



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

    private void threeSixPlanBlue() {
        intake.startIntakeWaitForBlock(gripperTrigger);
        //intake.startIntakeWaitForBlock(gripperTrigger);
        chassis.arc(25, 90, .3, false); //5000
        chassis.arc(-117, 4, .8, false); //500
        chassis.driveStraight(-50, .8); //1100

        /*chassis.pivot(-10, .4);
        chassis.driveStraight(550, 1);
        chassis.pivot(30, .5);
        intake.startIntakeWaitForBlock(ardennes.getIntakeTrigger());
        chassis.driveStraight(300, .3);
        */
    }

    private void oneFourPlanRed() {
        intake.startIntakeWaitForBlock(gripperTrigger);
        chassis.arc(30,70, .3, false); //2400
        chassis.arc(-58, 17, .8, true); //900
        chassis.driveStraight(-53,.8); //1000
    }

    private void twoFivePlanRed() {
        //intake.startIntakeWaitForBlock(gripperTrigger);
        intake.startIntakeWaitForBlock(gripperTrigger);
        chassis.arc(6,450, .3, false); //13000
        chassis.arc(-76,15,.8, true); //600
        chassis.driveStraight(-50, .8); //1100
    }

    private void threeSixPlanRed() {
        intake.startIntakeWaitForBlock(gripperTrigger);
        //intake.startIntakeWaitForBlock(gripperTrigger);
        chassis.arc(24, 100, .3, true); //5000
        chassis.arc(-103, 5, .8, true); //500
        chassis.driveStraight(-50, .8); //1100

    }

//    private void pivot(double angle, double maxPower) {
//        if (!redSide) {
//            chassis.pivot(-angle, maxPower);
//        } else {
//            chassis.pivot(angle, maxPower);
//        }
//
//    }
//
//    private void customArc(double angle, double radius, double maxPower, boolean rightTurn, double minRampUp, double minRampDown, long timeOut) {
//        if (!redSide) {
//            chassis.arc(angle, radius, maxPower, !rightTurn, minRampUp, minRampDown, timeOut);
//        } else {
//            chassis.arc(angle, radius, maxPower, rightTurn, minRampUp, minRampDown, timeOut);
//        }
//    }
//
//    private void arc(double angle, double radius, double maxPower, boolean rightTurn) {
//        if (!redSide) {
//            chassis.arc(angle, radius, maxPower, !rightTurn);
//        } else {
//            chassis.arc(angle, radius, maxPower, rightTurn);
//        }
//    }
}

