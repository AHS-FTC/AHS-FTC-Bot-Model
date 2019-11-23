package edu.ahs.robotics.hardware;


import edu.ahs.robotics.autocommands.PlanElement;

public interface Executor {
    public abstract void execute(PlanElement planElement);
}
