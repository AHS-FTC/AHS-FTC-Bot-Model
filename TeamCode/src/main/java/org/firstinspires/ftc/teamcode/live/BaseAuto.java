package org.firstinspires.ftc.teamcode.live;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.sql.Time;

import edu.ahs.robotics.control.MotionConfig;
import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.obm.BlockGripper;
import edu.ahs.robotics.control.obm.FoundationGrip;
import edu.ahs.robotics.control.obm.NullCommand;
import edu.ahs.robotics.control.obm.OBMCommand;
import edu.ahs.robotics.control.obm.GlobalHeadingChanger;
import edu.ahs.robotics.control.obm.SlideCycleDown;
import edu.ahs.robotics.control.obm.SlideCycleUp;
import edu.ahs.robotics.control.obm.TimedGripper;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.loggers.DataLogger;
import edu.ahs.robotics.util.loggers.Logger;

public class BaseAuto {
    public static final double MOTOR_POWER = 1;
    public static final double MOTOR_POWER_SLOW = .7;
    private Ardennes ardennes;
    private MecanumChassis chassis;
    private Intake intake;
    private Slides slides;

    private SerialServo leftFoundation, rightFoundation, xSlide, gripper, capstone;

    private Path quarry, toFoundation, gripFoundation, pullFoundation, quarry2, foundation2, quarry3, foundation3, quarry4, foundation4, quarry5, foundation5;

    private Logger logger;
    private OBMCommand nullCommand = new NullCommand();
    private SlideCycleUp slideCycleUp;
    private SlideCycleDown slideCycleDown;
    private BlockGripper blockGripper;
    private OBMCommand changeTargetHeading;
    private FoundationGrip foundationGrip;
    private TimedGripper timedGripper;

    private int turnSign;


    public BaseAuto(boolean flipToBlue) {


        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        intake = ardennes.getIntake();
        slides = ardennes.getSlides();

        leftFoundation = ardennes.getLeftFoundation();
        rightFoundation = ardennes.getRightFoundation();

        xSlide = ardennes.getxSlide();
        gripper = ardennes.getGripper();

        capstone = ardennes.getCapstone();

        xSlide.mapPosition(.3,.75);

        xSlide.setPosition(0);
        gripper.setPosition(0);

        leftFoundation.setPosition(0);
        rightFoundation.setPosition(0);

        capstone.setPosition(0.22);

        slideCycleUp = new SlideCycleUp(ardennes);
        slideCycleDown = new SlideCycleDown(ardennes);
        blockGripper = new BlockGripper(ardennes);
        foundationGrip = new FoundationGrip(ardennes, 900);
        timedGripper = new TimedGripper(ardennes, 500);

        if(flipToBlue){
            turnSign = -1;
        } else {
            turnSign = 1;
        }

        String pullFoundationName;
        String gripFoundationName;
        String parkName;

//        if (flipToBlue) {
//            pullFoundationName = "blue3-6-4_pullFoundation.csv";
//            gripFoundationName = "blue3-6-3_gripFoundation.csv";
//            parkName = "blue3-6-9_park.csv";
//        } else {
//            pullFoundationName = "3-6-4_pullFoundation.csv";
//            gripFoundationName = "3-6-3_gripFoundation.csv";
//            parkName = "3-6-9_park.csv";
//        }
//
//        pullFoundation = new Path(GCodeReader.openFile(pullFoundationName), 24, 24, 48, 28,2,10, false);
//        gripFoundation = new Path(GCodeReader.openFile(gripFoundationName), 12,12,12, false);
//        park = new Path(GCodeReader.openFile(parkName), 12,12,12, false);

        chassis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (flipToBlue) {
            chassis.setPosition(9,-38.25, 0);
        } else {
            chassis.setPosition(-9,-38.25, Math.PI);
        }


        String fileName = "partialPursuitAutoData";

        if (flipToBlue){
            fileName += "Blue";
        } else {
            fileName += "Red";
        }

        logger = new DataLogger(fileName, "partialPursuit");
        logger.startWriting();

        Logger odometryLogger = new DataLogger("odometryStats","odometrySystem"); //this bad boy is grabbed in OdometrySystemImpl

        chassis.startOdometrySystem();
    }

    public void setPaths(Path quarry, Path toFoundation, Path gripFoundation , Path pullFoundation, Path quarry2, Path foundation2, Path quarry3, Path foundation3, Path quarry4, Path foundation4, Path quarry5, Path foundation5) {
        this.quarry = quarry;
        this.toFoundation = toFoundation;
        this.gripFoundation = gripFoundation;
        this.pullFoundation = pullFoundation;
        this.quarry2 = quarry2;
        this.foundation2 = foundation2;
        this.quarry3 = quarry3;
        this.foundation3 = foundation3;
        this.quarry4 = quarry4;
        this.foundation4 = foundation4;
        this.quarry5 = quarry5;
        this.foundation5 = foundation5;
    }

    public void start(){
        intake.runMotors(MOTOR_POWER);

        MotionConfig quarryConfig = new MotionConfig();
        quarryConfig.idealHeading =  (turnSign)*(-Math.PI/3);
        quarryConfig.turnAggression = .8;
        quarryConfig.turnPower = .6;
        quarryConfig.timeOut = 4000;
        quarryConfig.turnCutoff = 2;
        quarryConfig.addOBMCommand(blockGripper);

        chassis.followPath(quarry,quarryConfig);

        slideCycleUp.setCycleHeight(220);

        MotionConfig toFoundationConfig = new MotionConfig();
        toFoundationConfig.idealHeading = Math.PI;
        toFoundationConfig.turnAggression = .9;
        toFoundationConfig.turnPower = 1;
        toFoundationConfig.timeOut = 5000;
//        changeTargetHeading = new GlobalHeadingChanger(toFoundationConfig, (turnSign) * Math.PI/2, 12);
//        toFoundationConfig.addOBMCommand(changeTargetHeading);
        toFoundationConfig.addOBMCommand(slideCycleUp);
        toFoundationConfig.addOBMCommand(timedGripper);
        toFoundationConfig.turnCutoff = 1;

        chassis.followPath(toFoundation, toFoundationConfig);

        //FOUNDATION

        leftFoundation.setPosition(.4);
        rightFoundation.setPosition(.4);

        MotionConfig gripFoundationConfig = new MotionConfig();
        gripFoundationConfig.timeOut = 5000;
        gripFoundationConfig.globalHeading = Math.PI;
        gripFoundationConfig.usingGlobalHeading = true;
        gripFoundationConfig.turnAggression = .2;
        gripFoundationConfig.turnPower = .7;
        gripFoundationConfig.addOBMCommand(slideCycleUp);
        gripFoundationConfig.addOBMCommand(foundationGrip);
        chassis.followPath(gripFoundation, gripFoundationConfig);

        chassis.setPowerAll(-.2);
        long startTime = FTCUtilities.getCurrentTimeMillis();
        while((FTCUtilities.getCurrentTimeMillis() - startTime) < 300){
            foundationGrip.check(null);
        }
        chassis.stopMotors();

        //leftFoundation.setPosition(1);
        //rightFoundation.setPosition(1);

        gripper.setPosition(0);
        //FTCUtilities.sleep(600);

        ardennes.finishOBMCommand(foundationGrip);

        MotionConfig pullFoundationConfig = new MotionConfig();
        pullFoundationConfig.addOBMCommand(slideCycleUp);
        pullFoundationConfig.timeOut = 3000;
        pullFoundationConfig.turnCutoff = 4.0;

        chassis.followPath(pullFoundation, pullFoundationConfig);

        leftFoundation.setPosition(0);
        rightFoundation.setPosition(0);

        intake.runMotors(MOTOR_POWER_SLOW);

        blockGripper.reset();
        timedGripper.reset();

        //BLOCK 2

        MotionConfig quarry2Config = new MotionConfig();
        quarry2Config.addOBMCommand(slideCycleDown);
        quarry2Config.addOBMCommand(blockGripper);
        quarry2Config.timeOut = 3500;
        quarry2Config.turnPower = .7;
        quarry2Config.turnAggression = .4;

        chassis.followPath(quarry2, quarry2Config);
        slideCycleUp.reset();
        slideCycleUp.setCycleHeight(400);
        slideCycleDown.reset();

        gripper.setPosition(1);

        MotionConfig foundation2Config = new MotionConfig();
        foundation2Config.idealHeading = Math.PI;
        foundation2Config.addOBMCommand(slideCycleUp);
        foundation2Config.addOBMCommand(timedGripper);
        foundation2Config.timeOut = 3000;

        chassis.followPath(foundation2, foundation2Config);
        chassis.stopMotors();

        gripper.setPosition(0);

        intake.runMotors(MOTOR_POWER);
        blockGripper.reset();
        timedGripper.reset();

        MotionConfig quarry3Config = new MotionConfig();
        quarry3Config.addOBMCommand(slideCycleDown);
        quarry3Config.addOBMCommand(blockGripper);
        quarry3Config.timeOut = 5000;

        chassis.followPath(quarry3, quarry3Config);

        slideCycleUp.reset();
        slideCycleUp.setCycleHeight(400);
        slideCycleDown.reset();

        //BLOCK 3

        MotionConfig foundation3Config = new MotionConfig();
        foundation3Config.idealHeading = Math.PI;
        foundation3Config.addOBMCommand(slideCycleUp);
        foundation3Config.addOBMCommand(timedGripper);
        foundation3Config.timeOut = 5000;
        foundation3Config.turnAggression = 1;
        foundation3Config.turnCutoff = 0;

        chassis.followPath(foundation3, foundation3Config);
        chassis.stopMotors();

        gripper.setPosition(0);

        intake.runMotors(MOTOR_POWER);
        blockGripper.reset();
        timedGripper.reset();

        //BLOCK 4

        MotionConfig quarry4Config = new MotionConfig();
        quarry4Config.addOBMCommand(slideCycleDown);
        quarry4Config.addOBMCommand(blockGripper);
        quarry4Config.timeOut = 5000;

        chassis.followPath(quarry4, quarry4Config);

        slideCycleUp.reset();
        slideCycleUp.setCycleHeight(400);
        slideCycleDown.reset();

        MotionConfig foundation4Config = new MotionConfig();
        foundation4Config.idealHeading = Math.PI;
        foundation4Config.addOBMCommand(slideCycleUp);
        foundation4Config.addOBMCommand(timedGripper);
        foundation4Config.timeOut = 5000;
        foundation4Config.turnAggression = 1;
        foundation4Config.turnCutoff = 0;

        chassis.followPath(foundation4, foundation4Config);
        chassis.stopMotors();

        gripper.setPosition(0);

        //BLOCK 5

        intake.runMotors(MOTOR_POWER_SLOW);
        blockGripper.reset();
        timedGripper.reset();

        MotionConfig quarry5Config = new MotionConfig();
        quarry5Config.addOBMCommand(slideCycleDown);
        quarry5Config.addOBMCommand(blockGripper);
        quarry5Config.timeOut = 3000;

        chassis.followPath(quarry5, quarry5Config);

        slideCycleUp.reset();
        slideCycleUp.setCycleHeight(400);

        MotionConfig foundation5Config = new MotionConfig();
        foundation5Config.idealHeading = Math.PI;
        foundation5Config.addOBMCommand(slideCycleUp);
        foundation5Config.addOBMCommand(timedGripper);
        foundation5Config.timeOut = 3000;
        foundation5Config.turnAggression = 1;
        foundation5Config.turnCutoff = 0;

        chassis.followPath(foundation5, foundation5Config);
        chassis.stopMotors();

        gripper.setPosition(0);

        //MotionConfig parkConfig = new MotionConfig();
        //parkConfig.addOBMCommand(tapeMeasure);
        //parkConfig.timeOut = 3000;

        //chassis.followPath(park, parkConfig);
        //chassis.stopMotors();
        //ardennes.finishOBMCommand(tapeMeasure);*/

        chassis.stopOdometrySystem();
        logger.stopWriting();
    }

}
