package edu.ahs.robotics.hardware;

import edu.ahs.robotics.control.Position;

public class PathTracker {
    private Position currentPosition;


    public PositionError getErrors(Position targetPosition, Position currentPosition) {
        //Do math to find error here

        return new PositionError();
    }

    public class PositionError {
        private double speedError;
        private double steeringError;

        public double getSpeedError() {
            return speedError;
        }

        public void setSpeedError(double speedError) {
            this.speedError = speedError;
        }

        public double getSteeringError() {
            return steeringError;
        }

        public void setSteeringError(double steeringError) {
            this.steeringError = steeringError;
        }
    }

}
