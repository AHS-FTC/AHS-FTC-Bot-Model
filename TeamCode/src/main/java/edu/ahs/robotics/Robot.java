
package edu.ahs.robotics;

import java.util.Iterator;

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



