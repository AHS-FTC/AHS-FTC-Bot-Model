package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.control.MotionConfig;
import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.obm.BlockGripper;
import edu.ahs.robotics.control.obm.NullCommand;
import edu.ahs.robotics.control.obm.OBMCommand;
import edu.ahs.robotics.control.obm.SlideCycle;
import edu.ahs.robotics.control.obm.TapeMeasureCommand;
import edu.ahs.robotics.hardware.ContinuosServo;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.DataLogger;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.GCodeReader;
import edu.ahs.robotics.util.Logger;

public class PartialPursuitAuto {
    private Ardennes ardennes;
    private MecanumChassis chassis;
    private Intake intake;
    private Slides slides;

    private SerialServo leftFoundation, rightFoundation, xSlide, gripper, capstone;

    private Path quarry, toFoundation, quarry2, foundation2, quarry3, foundation3;
    private Path gripFoundation, pullFoundation, park;

    private Logger logger;
    private OBMCommand nullCommand = new NullCommand();
    private SlideCycle slideCycle;
    private BlockGripper blockGripper;
    private OBMCommand tapeMeasure;

    private int turnSign;


    public PartialPursuitAuto(boolean flipToBlue) {
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        intake = ardennes.getIntake();
        slides = ardennes.getSlides();

        leftFoundation = ardennes.getLeftFoundation();
        rightFoundation = ardennes.getRightFoundation();

        xSlide = ardennes.getySlide();
        gripper = ardennes.getGripper();

        capstone = ardennes.getCapstone();

        xSlide.setPosition(0);
        gripper.setPosition(0);

        leftFoundation.setPosition(0);
        rightFoundation.setPosition(0);

        capstone.setPosition(0.22);

        slideCycle = new SlideCycle(ardennes);
        blockGripper = new BlockGripper(ardennes,500L);
        tapeMeasure = new TapeMeasureCommand(ardennes.getTapeMeasure());

        if(flipToBlue){
            turnSign = -1;
        } else {
            turnSign = 1;
        }

        chassis.setTriggerBoi(ardennes.getGripperTrigger());

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

        chassis.startOdometrySystem();
    }

    public void setPaths(Path quarry, Path toFoundation, Path quarry2, Path foundation2, Path quarry3, Path foundation3) {
        this.quarry = quarry;
        this.toFoundation = toFoundation;
        this.quarry2 = quarry2;
        this.foundation2 = foundation2;
        this.quarry3 = quarry3;
        this.foundation3 = foundation3;
    }

    public void start(){
        intake.runMotors(1);

        MotionConfig quarryConfig = new MotionConfig();
        quarryConfig.idealHeading =  (turnSign)*(-Math.PI/4);
        quarryConfig.timeOut = 4000;


        chassis.followPath(quarry,quarryConfig);

        MotionConfig toFoundationConfig = new MotionConfig();
        toFoundationConfig.idealHeading = Math.PI;
        toFoundationConfig.timeOut = 5000;
        toFoundationConfig.addOBMCommand(blockGripper);

        chassis.followPath(toFoundation, toFoundationConfig);

        chassis.stopMotors();

        double targetAngle = (2*Math.PI);
        if(turnSign == -1){
            targetAngle = -Math.PI;
        }

        chassis.globalPointTurn(targetAngle, 0.3, 1500);
        chassis.stopMotors();

        MotionConfig gripFoundationConfig = new MotionConfig();
        gripFoundationConfig.idealHeading = Math.PI;
        gripFoundationConfig.addOBMCommand(slideCycle);
        gripFoundationConfig.timeOut = 3000;
        gripFoundationConfig.turnCutoff = 20.0;

        chassis.followPath(gripFoundation,gripFoundationConfig);
        chassis.stopMotors();

        leftFoundation.setPosition(1);
        rightFoundation.setPosition(1);

        FTCUtilities.sleep(400);

        MotionConfig pullFoundationConfig = new MotionConfig();
        pullFoundationConfig.addOBMCommand(slideCycle);
        pullFoundationConfig.timeOut = 3000;
        pullFoundationConfig.turnCutoff = 4.0;

        chassis.followPath(pullFoundation, pullFoundationConfig);

        leftFoundation.setPosition(0);
        rightFoundation.setPosition(0);

        blockGripper.reset();
        blockGripper.resetWaitTime(10000L);

        intake.runMotors(1);

        MotionConfig quarry2Config = new MotionConfig();
        quarry2Config.addOBMCommand(slideCycle);
        quarry2Config.addOBMCommand(blockGripper);
        quarry2Config.timeOut = 3500;

        chassis.followPath(quarry2, quarry2Config);
        slideCycle.reset();
        slideCycle.setCycleHeight(280);

        blockGripper.resetWaitTime(500L);

        MotionConfig foundation2Config = new MotionConfig();
        foundation2Config.idealHeading = Math.PI;
        foundation2Config.addOBMCommand(blockGripper);
        foundation2Config.addOBMCommand(slideCycle);
        foundation2Config.timeOut = 3000;

        chassis.followPath(foundation2, foundation2Config);
        chassis.stopMotors();

        ardennes.finishOBMCommand(slideCycle);

        blockGripper.reset();
        blockGripper.resetWaitTime(10000L);

        intake.runMotors(1);

        MotionConfig quarry3Config = new MotionConfig();
        quarry3Config.addOBMCommand(blockGripper);
        quarry3Config.timeOut = 3500;

        chassis.followPath(quarry3, quarry3Config);

        slideCycle.reset();
        slideCycle.setCycleHeight(280);
        blockGripper.resetWaitTime(500L);

        MotionConfig foundation3Config = new MotionConfig();
        foundation3Config.idealHeading = Math.PI;
        foundation3Config.addOBMCommand(blockGripper);
        foundation3Config.addOBMCommand(slideCycle);
        foundation3Config.timeOut = 4000;

        chassis.followPath(foundation3, foundation3Config);
        chassis.stopMotors();

        ardennes.finishOBMCommand(slideCycle);

        MotionConfig parkConfig = new MotionConfig();
        parkConfig.addOBMCommand(tapeMeasure);
        parkConfig.timeOut = 3000;

        //chassis.followPath(park, parkConfig);
        chassis.stopMotors();
        //ardennes.finishOBMCommand(tapeMeasure);

        chassis.stopOdometrySystem();
        logger.stopWriting();
    }

}
