package edu.ahs.robotics.control;

import java.util.ArrayList;
import java.util.Iterator;

import edu.ahs.robotics.control.obm.NullCommand;
import edu.ahs.robotics.control.obm.OBMCommand;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;

/**
 * Class that contains info for followPath methods. Defaults are hard-coded in, so many things won't have to be changed.
 */
public class MotionConfig {
    public double lookAheadDistance = 12.0;
    public double idealHeading = 0.0;
    public double turnPower = 1.0;
    public double turnAggression = .8;
    public double turnCutoff = 8;

    public long timeOut =  10000L;//in milliseconds

    private ArrayList<OBMCommand> obmCommands = new ArrayList<>();

    public void addOBMCommand(OBMCommand obmCommand){
        obmCommands.add(obmCommand);
    }

    /**
     * Checks all OBMCommands on MotionConfig
     * @return to break the loop or not. If any OBMCommand returns true, this method returns true as well.
     */
    public boolean checkOBMCommands(OdometrySystem.State robotState){
        Iterator i = obmCommands.iterator();
        boolean finished = false;

        while (i.hasNext()){
            OBMCommand next = (OBMCommand)i.next();
            finished = finished || next.check(robotState); //if either is true, finished is true. also checks obmCommand here
        }

        return finished;
    }
}
