package edu.ahs.robotics.util;

import java.util.HashMap;
import java.util.Map;

/**
 * A class with static methods that returns motor statistics. Probably worth rewriting at some point
 */
public class MotorHashService {


    public enum MotorTypes{
        AM_20, YJ_223, YJ_435

    }

    private static Map<MotorTypes,Double> tickList;
    private static Map<MotorTypes, Double> rpmList;


    public static void init(){
        tickList = new HashMap<>();
        rpmList = new HashMap<>();
        tickList.put(MotorTypes.AM_20, 537.6);
        tickList.put(MotorTypes.YJ_223, 753.2);
        tickList.put(MotorTypes.YJ_435, 383.6);

        rpmList.put(MotorTypes.AM_20, 340.0);
        rpmList.put(MotorTypes.YJ_223, 223.0);
        rpmList.put(MotorTypes.YJ_435, 435.0);
    }

    public static double getTicks(MotorTypes motorType){
        return tickList.get(motorType);
    }

    public static double getRPMs(MotorTypes motorType) {
        return rpmList.get(motorType);
    }

}
