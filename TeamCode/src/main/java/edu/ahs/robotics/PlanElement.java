package edu.ahs.robotics;

public abstract class PlanElement {
    Executor executor;

    public PlanElement(Executor executor){
        this.executor = executor;
    }

    public void execute(){
        executor.execute(this);
    }
}
