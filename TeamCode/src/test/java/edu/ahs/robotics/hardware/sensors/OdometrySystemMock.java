package edu.ahs.robotics.hardware.sensors;

import java.util.List;

import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Velocity;

/**
 * High level mock for tests
 * @author Alex Appleby
 */
public class OdometrySystemMock implements OdometrySystem {
    private List<Position> positions;
    private List<Velocity> velocities;
    private boolean running;
    private int positionIndex; // a little bit sloppy but that's all cool. we gotta avoid them threads.
    private int velocityIndex; // this also allows you to not give a shit about velocity or position without exploding everything

    public OdometrySystemMock(List<Position> positions, List<Velocity> velocities) {
        this.positions = positions;
        this.velocities = velocities;

        positionIndex = 0;
        velocityIndex = 0;

        running = false;
    }

    @Override
    public Position getPosition() {
        Position position = positions.get(positionIndex);
        positionIndex ++;
        return position;
    }

    @Override
    public Velocity getVelocity() {
        Velocity velocity = velocities.get(velocityIndex);
        velocityIndex ++;
        return velocity;
    }

    @Override
    public void start() {
        running = true;
    }

    @Override
    public void stop() {
        running = false;
    }

    @Override
    public boolean isRunning() {
        return running;
    }

    @Override
    public void setPosition(double x, double y, double heading){
        //literally nobody cares about this method
    }
}
