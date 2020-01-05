package edu.ahs.robotics.seasonrobots;


import java.util.HashMap;
import java.util.Map;

import edu.ahs.robotics.hardware.ChassisMotors;
import edu.ahs.robotics.hardware.DriveUnit;
import edu.ahs.robotics.hardware.GearRatio;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.Robot;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.sensors.ArdennesSkyStoneDetector;
import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.hardware.sensors.OdometrySystemImpl;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;
import edu.ahs.robotics.hardware.Slides;


public class Ardennes extends Robot {
    private MecanumChassis mecanumChassis;
    private OdometrySystemImpl odometrySystem;
    private Intake intake;
    private SerialServo gripper;//todo add other serbos
    private TriggerDistanceSensor intakeTrigger, gripperTrigger;
    private ArdennesSkyStoneDetector detector;
    private Slides slides;
    private SerialServo leftFoundation, rightFoundation;
    private SerialServo ySlide;
    private SerialServo wrist;
    private SerialServo intakeServo;


    public Ardennes() {
        intakeTrigger = new TriggerDistanceSensor("intakeTrigger",70, 300);
        gripperTrigger = new TriggerDistanceSensor("gripperTrigger", 40, 0);
        leftFoundation = new SerialServo("FSL", true);
        rightFoundation = new SerialServo("FSR", false);
        intake = new Intake(1);
        gripper = new SerialServo("gripper", true);
        odometrySystem = makeOdometrySystem();
        mecanumChassis = makeChassis(odometrySystem);
        slides = new Slides();
        ySlide = new SerialServo("slideServo", false);
        wrist = new SerialServo("wrist", true);
        intakeServo = new SerialServo("intakeLock", false);
    }

    public Intake getIntake(){
        return intake;
    }

    public MecanumChassis getChassis(){
        return mecanumChassis;
    }

    public TriggerDistanceSensor getIntakeTrigger() {
        return intakeTrigger;
    }

    public TriggerDistanceSensor getGripperTrigger() {
        return gripperTrigger;
    }

    public SerialServo getGripper(){
        return gripper;
    }

    public SerialServo getIntakeServo() {return intakeServo;}

    public SerialServo getLeftFoundation() {return leftFoundation;}
    public SerialServo getRightFoundation() {return rightFoundation;}

    public SerialServo getySlide() {return ySlide;}

    public Slides getSlides() {return slides;}

    public SerialServo getWrist() {return wrist;}

    private MecanumChassis makeChassis(OdometrySystemImpl odometrySystem) {
        //Set Gear Ratio
        GearRatio driveGearRatio = new GearRatio(1,1);
        //Set Wheel Diameter in inches and Motor Type. These traits are shared by all chassis drive units
        DriveUnit.Config config = new DriveUnit.Config(driveGearRatio, 3.94, MotorHashService.MotorTypes.AM_20);

        //Make a HashMap that maps motors to their flip status. True indicates the motor runs reverse.
        Map<ChassisMotors.Mecanum, Boolean> driveFlips  = new HashMap<>();

        //OdometrySystemImpl odometrySystem = makeOdometrySystem(imu);

        return new MecanumChassis(config, odometrySystem);
    }

    private OdometrySystemImpl makeOdometrySystem(){
        Odometer x1 = FTCUtilities.getOdometer("intakeL", 2.3596,true,1440.0); //2.3596 //*** IMPORTANT *** setDirection() method on DcMotor changes encoder direction
        Odometer x2 = FTCUtilities.getOdometer("intakeR", 2.3617, true,1440.0); //2.3617
        Odometer y = FTCUtilities.getOdometer("BR", 2.387, false,4000);

        OdometrySystemImpl odometrySystem = new OdometrySystemImpl(x1, x2, y, -.1144, 14.1);
        return odometrySystem;
    }

}
