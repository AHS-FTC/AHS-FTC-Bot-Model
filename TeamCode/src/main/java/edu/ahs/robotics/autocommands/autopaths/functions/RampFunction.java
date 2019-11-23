package edu.ahs.robotics.autocommands.autopaths.functions;

public class RampFunction implements Function {
    private double distance;
    private double executionTime;
    private double H_DILATION=0.005;
    private double V_DILATION=20;
    private double OFFSET = 750;

    /**
     *
     * @param distance in inches
     */
    public RampFunction(double distance){
        this.distance = distance;
        this.executionTime=Math.log(Math.pow(10,distance/V_DILATION)-1)/H_DILATION+2*OFFSET;
    }


    @Override
    //This method takes a time in miliseconds and returns the desired targetDistance in inches
    public double getDesiredDistance(double time) {
        if(time<executionTime/2) {
            return fx(time);
        }
        else if (time<executionTime){
            return gx(time);
        }
        else {return gx(executionTime);
        }
    }

    private double fx(double time){
        return V_DILATION*Math.log10(1+Math.exp(H_DILATION*(time-OFFSET)));
    }

    private double gx(double time){
        return -V_DILATION*Math.log10(1+Math.exp(-H_DILATION*(time-executionTime+OFFSET)))+2*fx(executionTime/2);
    }



}
