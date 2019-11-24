package edu.ahs.robotics.abbreviatedmodel;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.Iterator;

import edu.ahs.robotics.autocommands.Plan;
import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.autocommands.autopaths.Sleep;
import edu.ahs.robotics.hardware.Executor;
import edu.ahs.robotics.util.FTCUtilities;

public abstract class Robot implements Executor{
    private Plan plan;

    public void givePlan (Plan plan) {
        this.plan = plan;
    }

    public void execute(PlanElement planElement) {
        if(planElement instanceof Sleep){
            Sleep sleep = (Sleep)planElement;
            double startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < sleep.time){
                FTCUtilities.OpLogger("sleeping for", System.currentTimeMillis() - startTime);
            }
        } else{
            throw new Warning("Could not executePlan planElement " + planElement.toString() + " in Robot class");
        }
    }
    public void executePlan() {
        FTCUtilities.OpLogger("Executing","In Robot");
        Iterator<PlanElement> iterator = plan.getIterator();
        while(iterator.hasNext()){
            PlanElement element = iterator.next();
            element.execute();
        }
    }
}
