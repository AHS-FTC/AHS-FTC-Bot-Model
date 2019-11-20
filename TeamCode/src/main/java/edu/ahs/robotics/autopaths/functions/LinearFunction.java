package edu.ahs.robotics.autopaths.functions;

public class LinearFunction implements Function {

    private double distance;
    private double executionTime;
    private double maxSpeed;

    /**
     *
     * @param distance in inches
     * @param maxSpeed in inches/second
     */
    public LinearFunction(double distance, double maxSpeed){
        this.distance = distance;
        this.maxSpeed = maxSpeed/1000;
        this.executionTime = distance/this.maxSpeed;

    }


    @Override
    //This method returns the desired distance in inches
    public double getDesiredDistance(double time) {
        if(time<executionTime) {
            return maxSpeed * time;
        }
        else {return maxSpeed*executionTime;}
    }
}
