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
    private TriggerDistanceSensor gripperTrigger;
    private TriggerDistanceSensor intakeTrigger;
    private ArdennesSkyStoneDetector detector;
    private boolean mirrored;
    private ArdennesSkyStoneDetector.SkyStoneConfigurations stoneConfiguration;

    public FullAuto(boolean mirrored) {
        this.mirrored = mirrored;
    }

    public void init() {

        MotorHashService.init();
        ardennes = new Ardennes();
        detector = new ArdennesSkyStoneDetector();
        intake = ardennes.getIntake();
        chassis = ardennes.getChassis();
        slides = ardennes.getSlides();
        foundationServoLeft = ardennes.getLeftFoundation();
        foundationServoRight = ardennes.getRightFoundation();
        gripper = ardennes.getGripper();
        intakeServo = ardennes.getIntakeServo();
        yslide = ardennes.getySlide();
        gripperTrigger = ardennes.getGripperTrigger();
        intakeTrigger = ardennes.getIntakeTrigger();
        slides.resetEncoders();
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

//        stoneConfiguration = detector.look(mirrored);
//
//        if (ArdennesSkyStoneDetector.SkyStoneConfigurations.ONE_FOUR == stoneConfiguration) {
//            leftPlan();
//        } else if (ArdennesSkyStoneDetector.SkyStoneConfigurations.TWO_FIVE == stoneConfiguration) {
//            middlePlan();
//        } else rightPlan();
        intake.startIntakeWaitForBlock(intakeTrigger);
        arc(40,1450, .65, false);
        arc(-46, 1500, .8, true);
        chassis.driveStraight(-800,.8);
        FTCUtilities.sleep(300);
        pivot(-90, .7);
        FTCUtilities.sleep(300);
        chassis.driveStraight(-200,.8);
        foundationServoLeft.setPosition(1);
        foundationServoRight.setPosition(1);
        FTCUtilities.sleep(500);
        arc(90,80,1,true);
        FTCUtilities.sleep(300);
        chassis.driveStraight(-200, 1);

    }

    private void leftPlan() {
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
        arc(40,1450, .65, false);
        arc(-46, 1500, .8, true);
        chassis.driveStraight(-800,.8);
        FTCUtilities.sleep(300);
        pivot(-90, .7);
        FTCUtilities.sleep(300);
        chassis.driveStraight(-200,.8);
        foundationServoLeft.setPosition(1);
        foundationServoRight.setPosition(1);
        FTCUtilities.sleep(500);
        arc(90,80,1,true);
        FTCUtilities.sleep(300);
        chassis.driveStraight(-200, 1);


//        arc(90, 300, .93, true);

        /*chassis.driveStraight(500, 1);
        chassis.pivot(-30, .4);
        intake.startIntakeWaitForBlock(ardennes.getIntakeTrigger());
        chassis.driveStraight(450, .3);
        */

    }

    private void middlePlan() {
        pivot(-9, .93);
        intake.startIntakeWaitForBlock(gripperTrigger);
        chassis.driveStraight(900, .7);

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

    private void rightPlan() {
        pivot(5, .93);
        intake.startIntakeWaitForBlock(gripperTrigger);
        chassis.driveStraight(950, .7);

        /*chassis.pivot(-10, .4);
        chassis.driveStraight(550, 1);
        chassis.pivot(30, .5);
        intake.startIntakeWaitForBlock(ardennes.getIntakeTrigger());
        chassis.driveStraight(300, .3);
        */

    }

    private void pivot(double angle, double maxPower) {
        if (mirrored) {
            chassis.pivot(-angle, maxPower);
        } else {
            chassis.pivot(angle, maxPower);
        }

    }

    private void arc(double angle, double radius, double maxPower, boolean rightTurn) {
        if (mirrored) {
            chassis.arc(angle, radius, maxPower, !rightTurn);
        } else {
            chassis.arc(angle, radius, maxPower, rightTurn);
        }
    }
}

