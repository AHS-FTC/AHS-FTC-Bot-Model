
package edu.ahs.robotics.hardware;

import java.util.Iterator;

import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.autopaths.Plan;
import edu.ahs.robotics.autopaths.PlanElement;

public class Robot {

    private Plan plan;
    private Chassis chassis;

    public Robot(Chassis chassis) {
        this.chassis = chassis;
    }

    public void givePlan (Plan plan) {
        this.plan = plan;
    }

    public void execute() {
        FTCUtilities.OpLogger("Executing","In Robot");
        Iterator<PlanElement> iterator = plan.getIterator();
        while(iterator.hasNext()){
            PlanElement element = iterator.next();
            element.execute();
        }
    }

    public Chassis getChassis(){
        return chassis;
    }

}



