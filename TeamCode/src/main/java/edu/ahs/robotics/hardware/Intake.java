package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSenor;
import edu.ahs.robotics.util.FTCUtilities;

public class Intake implements Executor{
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private TriggerDistanceSenor trigger;
    private IntakeMode intakeMode;

    public enum IntakeMode {
        OFF,
        IN,
        OUT
    }

    public Intake(TriggerDistanceSenor trigger) {
        leftMotor = FTCUtilities.getMotor("intakeL");
        rightMotor = FTCUtilities.getMotor("intakeR");
        this.trigger = trigger;
        intakeMode = IntakeMode.OFF;
    }

    public Intake(){
        this(null);
    }

    public void execute(PlanElement planElement){

    }

    private void execute(){

    }
}
