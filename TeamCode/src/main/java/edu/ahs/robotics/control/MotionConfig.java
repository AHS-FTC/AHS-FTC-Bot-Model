package edu.ahs.robotics.control;

import edu.ahs.robotics.control.obm.NullCommand;
import edu.ahs.robotics.control.obm.OBMCommand;

/**
 * Class that contains info for followPath methods. Defaults are hard-coded in, so many things won't have to be changed.
 */
public class MotionConfig {
    public double lookAheadDistance = 12.0;
    public double idealHeading = 0.0;
    public double turnPower = 1.0;
    public double turnAggression = .8;
    public double turnCutoff = 8;

    public OBMCommand obmCommand = new NullCommand(); //does nothing by default
    public OBMCommand obmCommand2 = new NullCommand();
    public long timeOut =  10000L;//in milliseconds
}
