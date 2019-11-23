package edu.ahs.robotics.hardware;

import edu.ahs.robotics.autocommands.PlanElement;

/**
 * The Chassis class is the superclass for all chassis types
 */
public abstract class Chassis implements Executor {

    public Chassis(){
    }
    public abstract void execute(PlanElement planElement);

}

