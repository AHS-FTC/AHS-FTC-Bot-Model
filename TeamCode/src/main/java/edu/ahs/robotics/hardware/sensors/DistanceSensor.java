package edu.ahs.robotics.hardware.sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.ahs.robotics.util.ftc.FTCUtilities;

public class DistanceSensor {

    private Rev2mDistanceSensor distanceSensor;
    private DistanceUnit distanceUnit;
    private long updateTime = 100;
    private long lastTime;
    private double previousDistance;

    public DistanceSensor(String sensorName) {
        this(sensorName, DistanceUnit.MM);
    }

    public DistanceSensor(String sensorName, DistanceUnit distanceUnit){
        distanceSensor = FTCUtilities.getDistanceSensor(sensorName);
        this.distanceUnit = distanceUnit;
        lastTime = -updateTime; //ensures that first call to getDistOptimized queries
    }

    public double getDist(){
        return distanceSensor.getDistance(distanceUnit);
    }

    /**
     * Gets distance only when time from previous sensor query exceeds updateTime. Otherwise returns previous query result.
     */
    public double getDistOptimized (){
        long timeSinceLast = FTCUtilities.getCurrentTimeMillis() - lastTime;

        if(timeSinceLast > updateTime){
            previousDistance = getDist();
            lastTime = FTCUtilities.getCurrentTimeMillis();
        }

        return previousDistance;
    }

    public void setUpdateTime(long updateTime){
        this.updateTime = updateTime;
    }

}
