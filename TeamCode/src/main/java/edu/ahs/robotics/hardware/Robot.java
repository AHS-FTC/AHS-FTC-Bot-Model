
package edu.ahs.robotics.hardware;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import edu.ahs.robotics.autocommands.autopaths.Sleep;
import edu.ahs.robotics.botfactories.BotFactory;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.autocommands.Plan;
import edu.ahs.robotics.autocommands.PlanElement;

public class Robot implements Executor{


    @Override
    public void execute(PlanElement planElement) {
        if(planElement instanceof Sleep){
            Sleep sleep = (Sleep)planElement;
            double startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < sleep.time){
                FTCUtilities.OpLogger("sleeping for", System.currentTimeMillis() - startTime);
            }
        } else{
            throw new Warning("Could not execute planElement " + planElement.toString() + " in Robot class");
        }
    }

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



