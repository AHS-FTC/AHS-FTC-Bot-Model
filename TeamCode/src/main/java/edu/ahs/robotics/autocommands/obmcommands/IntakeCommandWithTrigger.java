package edu.ahs.robotics.autocommands.obmcommands;

import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.hardware.Executor;
import edu.ahs.robotics.hardware.sensors.Trigger;

public class IntakeCommandWithTrigger extends PlanElement {
    public Trigger trigger;

    public IntakeCommandWithTrigger(Executor executor, Trigger trigger) {
        super(executor);
        this.trigger = trigger;
    }
}
