package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.obm.BlockGripper;
import edu.ahs.robotics.control.obm.NullCommand;
import edu.ahs.robotics.control.obm.OBMCommand;
import edu.ahs.robotics.control.obm.SlideCycle;
import edu.ahs.robotics.hardware.ContinuosServo;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.GCodeReader;
import edu.ahs.robotics.util.Logger;

public class PartialPursuitAuto {
    private Ardennes ardennes;
    private MecanumChassis chassis;
    private Intake intake;
    private Slides slides;

    private SerialServo leftFoundation, rightFoundation, xSlide, gripper;
    private ContinuosServo tapeMeasure;

    private Path quarry, toFoundation, quarry2, foundation2, quarry3, foundation3;
    private Path gripFoundation, pullFoundation, scoreFoundation;

    private Logger logger;
    private OBMCommand nullCommand = new NullCommand();
    private SlideCycle slideCycle, cycle2;
    private OBMCommand blockGripper;

    private int turnSign;


    public PartialPursuitAuto(Path quarry, Path toFoundation, Path quarry2, Path foundation2, Path quarry3, Path foundation3, boolean flipToBlue) {
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        intake = ardennes.getIntake();
        slides = ardennes.getSlides();

        leftFoundation = ardennes.getLeftFoundation();
        rightFoundation = ardennes.getRightFoundation();

        tapeMeasure = ardennes.getTapeMeasure();

        xSlide = ardennes.getySlide();
        gripper = ardennes.getGripper();

        xSlide.setPosition(0);
        gripper.setPosition(0);

        leftFoundation.setPosition(0);
        rightFoundation.setPosition(0);

        this.quarry = quarry;
        this.toFoundation = toFoundation;
        this.quarry2 = quarry2;
        this.foundation2 = foundation2;
        this.quarry3 = quarry3;
        this.foundation3 = foundation3;

        slideCycle = new SlideCycle(ardennes);
        blockGripper = new BlockGripper(ardennes,500L);

        if(flipToBlue){
            turnSign = -1;
        } else {
            turnSign = 1;
        }

        String pullFoundationName;
        String gripFoundationName;
        String scoreFoundationName;

        if (flipToBlue) {
            pullFoundationName = "blue3-6-4_pullFoundation.csv";
            gripFoundationName = "blue3-6-3_gripFoundation.csv";
            scoreFoundationName = "blue3-6-7_scoreFoundation.csv";
        } else {
            pullFoundationName = "3-6-4_pullFoundation.csv";
            gripFoundationName = "3-6-3_gripFoundation.csv";
            scoreFoundationName = "3-6-7_scoreFoundation.csv";
        }

        pullFoundation = new Path(GCodeReader.openFile(pullFoundationName), 24, 24, 48, 28,2,10, false);
        gripFoundation = new Path(GCodeReader.openFile(gripFoundationName), 12,12,12, false);

        //scoreFoundation = new Path(GCodeReader.openFile(scoreFoundationName), 12,12,12, false);

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

        logger = new Logger(fileName, "partialPursuit");
        logger.startWriting();

        chassis.startOdometrySystem();
    }

    public void start(){
        intake.runMotors(1);
        chassis.followPath(quarry, 12, (turnSign)*(-Math.PI/4), nullCommand,10000, 8);
        chassis.followPath(toFoundation, 12, Math.PI, blockGripper,10000, 8);
        chassis.stopMotors();

        double targetAngle = (2*Math.PI);
        if(turnSign == -1){
            targetAngle = -Math.PI;
        }

        chassis.globalPointTurn(targetAngle, 0.3, 2500);
        chassis.stopMotors();

        chassis.followPath(gripFoundation, 12, Math.PI, slideCycle,10000, 20);
        chassis.stopMotors();

        leftFoundation.setPosition(1);
        rightFoundation.setPosition(1);

        FTCUtilities.sleep(400);

        chassis.followPath(pullFoundation, 12, 0, slideCycle,10000, 4);

        leftFoundation.setPosition(0);
        rightFoundation.setPosition(0);

        blockGripper.reset();

        intake.runMotors(1);
        chassis.followPath(quarry2, 12, 0, slideCycle,10000, 8);
        slideCycle.reset();
        slideCycle.setCycleHeight(200);
        chassis.followPath(foundation2,12, Math.PI, blockGripper, slideCycle,10000, 8);
        chassis.stopMotors();

        ardennes.finishOBMCommand(slideCycle);

        blockGripper.reset();

        intake.runMotors(1);
        chassis.followPath(quarry3, 12, 0, nullCommand, 10000, 8);
        slideCycle.reset();
        chassis.followPath(foundation3, 12, Math.PI, blockGripper, slideCycle, 10000, 8);
        chassis.stopMotors();

//        chassis.followPath(scoreFoundation, 12, Math.PI, cycle2,4000);
//        chassis.stopMotors();
//
//        long startTime = FTCUtilities.getCurrentTimeMillis();
//
//        chassis.drive3Axis(1,0,0);
//        tapeMeasure.setPower(-.85);
//        while (FTCUtilities.getCurrentTimeMillis() - startTime < 670){
//
//        }
//        chassis.stopMotors();
//        tapeMeasure.setPower(0);

        slides.stopMotors();
        chassis.stopMotors();
        intake.stopMotors();
        chassis.stopOdometrySystem();
        logger.stopWriting();
    }

}
