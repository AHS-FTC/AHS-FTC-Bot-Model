package edu.ahs.robotics;

/**
 * The Chassis class is the superclass for all chassis types
 */
public abstract class Chassis implements Executor{

    public Chassis(BotFactory botFactory){
    }
    public abstract void execute(PlanElement planElement);

}

