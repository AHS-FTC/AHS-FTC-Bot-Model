package edu.ahs.robotics.hardware.sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.ahs.robotics.util.FTCUtilities;

public class DistanceSensor {
    Rev2mDistanceSensor distanceSensor;
    DistanceUnit distanceUnit;
    //double toggleDistance;

    public DistanceSensor(String sensorName) {
        this(sensorName, DistanceUnit.MM);
    }

    public DistanceSensor(String sensorName, DistanceUnit distanceUnit){
        distanceSensor = FTCUtilities.getDistanceSensor(sensorName);
        this.distanceUnit = distanceUnit;
    }
    //public void setToggleDistance(double toggleDistance){
       // this.toggleDistance = toggleDistance;
    //}

    public double getDist(){
        return distanceSensor.getDistance(distanceUnit);
    }

//    public boolean isEngaged(){
//        if (getDist()<=toggleDistance){
//            return true;
//        }else{
//            return false;
//        }
//
//    }
}
