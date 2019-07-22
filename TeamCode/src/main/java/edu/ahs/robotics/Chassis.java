package edu.ahs.robotics;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

/**
 * The Chassis class is the superclass for all chassis types
 */
public abstract class Chassis implements Executor{

    public Chassis(BotConfig botConfig){
    }
    public abstract void execute(PlanElement planElement);

}

