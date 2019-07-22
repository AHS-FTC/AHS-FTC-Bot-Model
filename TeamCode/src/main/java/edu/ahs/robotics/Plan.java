package edu.ahs.robotics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Plan {
    private List<PlanElement> plan;

    public Plan () {
        this.plan = new ArrayList<>();
    }

    public void addToPlan(PlanElement element) {
        plan.add(element);
    }

    public PlanElement getFromPlan(int index) {
        return plan.get(index);
    }

    public int planLength() {
        return plan.size();
    }

    public List<PlanElement> getPlan() {
        return plan;
    }

    public void logPlan(OpMode opMode){
        for(PlanElement i : plan){
            //opMode.telemetry.addData(i);
        }
    }
    public Iterator<PlanElement> getIterator(){
        return plan.iterator();
    }

}
