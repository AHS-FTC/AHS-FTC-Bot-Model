package edu.ahs.robotics.autocommands.obmcommands;

import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.hardware.Executor;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;

public class IntakeCommandWithTrigger extends PlanElement {
    public TriggerDistanceSensor trigger;

    public IntakeCommandWithTrigger(Executor executor, TriggerDistanceSensor trigger) {
        super(executor);
        this.trigger = trigger;
    }
}
