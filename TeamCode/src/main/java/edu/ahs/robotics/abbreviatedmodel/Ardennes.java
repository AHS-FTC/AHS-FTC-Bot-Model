package edu.ahs.robotics.abbreviatedmodel;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import edu.ahs.robotics.autocommands.Plan;
import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.autocommands.autopaths.Sleep;
import edu.ahs.robotics.hardware.ChassisMotors;
import edu.ahs.robotics.hardware.DriveUnit;
import edu.ahs.robotics.hardware.Executor;
import edu.ahs.robotics.hardware.GearRatio;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.sensors.IMU;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;

public class Ardennes extends Robot {
    private MecanumChassis mecanumChassis;
    private Intake intake;
    private TriggerDistanceSensor intakeTrigger, gripperTrigger;



    public Ardennes() {
        intakeTrigger = new TriggerDistanceSensor("intakeTrigger",70);
        gripperTrigger = new TriggerDistanceSensor("gripperTrigger", 40);

        intake = new Intake(1);
        mecanumChassis = makeChassis();
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
