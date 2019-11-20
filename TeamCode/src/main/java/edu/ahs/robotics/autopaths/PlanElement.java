package edu.ahs.robotics.autopaths;

import edu.ahs.robotics.hardware.Executor;

public abstract class PlanElement {
    Executor executor;

    public PlanElement(Executor executor){
        this.executor = executor;
    }

    public void execute(){
        executor.execute(this);
    }
}
