package edu.ahs.robotics.autopaths.functions;

public class Position {
    public double x;
    public double y;
    public double heading;

    public Position(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void setPosition(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
