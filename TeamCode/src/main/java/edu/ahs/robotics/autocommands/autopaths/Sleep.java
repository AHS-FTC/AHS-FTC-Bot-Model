package edu.ahs.robotics.autocommands.autopaths;

import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.hardware.Executor;
import edu.ahs.robotics.hardware.Robot;

public class Sleep extends PlanElement {
    public double time;

    public Sleep(Executor executor, double time){
        super(executor);
        if (!(executor instanceof Robot)){
            try {
                throw new Exception("The executor of a Sleep class should be of instance Robot");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        this.time = time;
    }
}
