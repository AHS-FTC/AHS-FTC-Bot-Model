
package edu.ahs.robotics;

import java.util.Iterator;

public class Robot {

    private Plan plan;
    private Chassis chassis;

    public Robot(BotFactory botFactory){

        if(!botFactory.isFullyConfigured()) {
            throw new Error("You haven't fully configured your robot");
        }
        if (botFactory.getChassisType() == MecanumChassis.class){
            chassis = new MecanumChassis(botFactory);
        }
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



