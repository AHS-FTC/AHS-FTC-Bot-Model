package edu.ahs.robotics.autocommands.obmcommands;

import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.hardware.Executor;

public class ServoCommand extends PlanElement {
    public double position;

    public ServoCommand(Executor executor, double position) {
        super(executor);
        this.position = position;
    }
}
