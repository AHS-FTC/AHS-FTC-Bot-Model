package edu.ahs.robotics.hardware.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.autocommands.autopaths.functions.Position;
import edu.ahs.robotics.util.FTCUtilities;

/**
 * A collection of Odometers used to monitor robot position. Written for Ardennes in 2019-20
 * @author Alex Appleby
 */
public class OdometrySystem {
    private IMU imu;
    private Position position;
    Odometer y1, y2, x;
    double y1Last, y2Last, xLast;


    /**
     * @param y1 The 'first' odometer measuring in the Y direction. Should be interchangeable with y2
     * @param y2 The 'second' odometer measuring in the Y direction. Should be interchangeable with y1
     * @param x The 'odometer measuring in the X direction.
     */
    public OdometrySystem(Odometer y1, Odometer y2, Odometer x, double xInchesPerDegree) {
        this.y1 = y1;
        this.y2 = y2;
        this.x =x;
        position = new Position(0,0,0);

        y1Last = y1.getDistance();
        y2Last = y2.getDistance();
        xLast = x.getDistance();
    }

    /**
     * starts thread continuously monitoring position
     */
    public void start(){

    }

    public void resetPosition(double x, double y, double heading){
        position.x = x;
        position.y = y;
        position.heading = heading;
    }

    /**
     * Runs central odom math, called continuously by thread and accessible in package for unit testing
     */
    void updatePosition() {
        // Read the sensor values
        // Calculate the new position
    }

    public Position getPosition(){
        return position;
    }

    private void findHeading(){

    }

}
