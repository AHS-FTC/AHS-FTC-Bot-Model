package edu.ahs.robotics.seasonrobots;

import com.qualcomm.hardware.bosch.BNO055IMU;

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
import edu.ahs.robotics.hardware.sensors.IMU;
import edu.ahs.robotics.hardware.sensors.LimitSwitch;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;
import edu.ahs.robotics.hardware.Slides;


public class Ardennes extends Robot {
    private MecanumChassis mecanumChassis;
    private Intake intake;
    private SerialServo gripper;//todo add other serbos
    private TriggerDistanceSensor intakeTrigger, gripperTrigger;
    private LimitSwitch limitSwitch;
    private ArdennesSkyStoneDetector detector;
    private Slides slides;
    private SerialServo leftFoundation, rightFoundation;
    private SerialServo ySlide;


    public Ardennes() {
        intakeTrigger = new TriggerDistanceSensor("intakeTrigger",70);
        gripperTrigger = new TriggerDistanceSensor("gripperTrigger", 40);
        limitSwitch = new LimitSwitch("limitSwitch");
        leftFoundation = new SerialServo("FSL", false);
        rightFoundation = new SerialServo("FSR", false);
        intake = new Intake(1);
        gripper = new SerialServo("gripper", true);
        mecanumChassis = makeChassis();
        slides = new Slides(.8, limitSwitch);
        detector = new ArdennesSkyStoneDetector();
        ySlide = new SerialServo("slideServo", false);
    }

    public ArdennesSkyStoneDetector.SkyStoneConfigurations runDetector() {return detector.look();}

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

    public LimitSwitch getLimitSwitch(){
        return limitSwitch;
    }

    public SerialServo getGripper(){
        return gripper;
    }

    public SerialServo getLeftFoundation() {return leftFoundation;}
    public SerialServo getRightFoundation() {return rightFoundation;}

    public SerialServo getySlide() {return ySlide;}

    public Slides getSlides() {return slides;}

    private MecanumChassis makeChassis() {
        //Set Gear Ratio
        GearRatio driveGearRatio = new GearRatio(1,1);
        //Set Wheel Diameter in inches and Motor Type. These traits are shared by all chassis drive units
        DriveUnit.Config config = new DriveUnit.Config(driveGearRatio, 3.94, MotorHashService.MotorTypes.AM_20);

        //Make a HashMap that maps motors to their flip status. True indicates the motor runs reverse.
        Map<ChassisMotors.Mecanum, Boolean> driveFlips  = new HashMap<>();

        BNO055IMU bnoImu = FTCUtilities.getIMU("imu");
        IMU imu = new IMU(bnoImu);
        //OdometrySystem odometrySystem = makeOdometrySystem(imu);

        driveFlips.put(ChassisMotors.Mecanum.FRONTLEFT, false); //todo check directions
        driveFlips.put(ChassisMotors.Mecanum.FRONTRIGHT, true);
        driveFlips.put(ChassisMotors.Mecanum.BACKLEFT, false);
        driveFlips.put(ChassisMotors.Mecanum.BACKRIGHT, true);

        return new MecanumChassis(config, driveFlips, imu);
    }



}
