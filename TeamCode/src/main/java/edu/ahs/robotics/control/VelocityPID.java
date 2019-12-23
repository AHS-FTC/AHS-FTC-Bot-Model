package edu.ahs.robotics.control;

public class VelocityPID{
    Config config;
    private double speedErrorSum = 0, directionErrorSum = 0;
    private double lastSpeedError = 0, lastDirectionError = 0;

    public VelocityPID(Config config) {
        this.config = config;
    }

    public Correction getCorrection(Velocity currentVelocity, Velocity targetVelocity){
        double speedCorrection = 0, directionCorrection = 0;

        double speedError = targetVelocity.speed - currentVelocity.speed;
        double directionError = targetVelocity.direction - currentVelocity.direction;

        speedCorrection += speedError * config.sP; //proportional
        directionCorrection += directionError * config.dP;

        speedErrorSum += speedError;
        directionErrorSum += directionError;

        speedCorrection += speedErrorSum * config.sI; //integral
        directionCorrection += directionErrorSum * config.dI;

        speedCorrection += (speedError - lastSpeedError) * config.sD; //derivative
        directionCorrection += (directionError - lastDirectionError) * config.dD;

        lastSpeedError = speedError; //update lasts
        lastDirectionError = directionError;

        return new Correction(speedCorrection, directionCorrection);
    }

    public static class Config{
        private double sP, sI, sD;
        private double dP, dI, dD;

        public void setSpeedParams(double p, double i, double d){
            sP = p;
            sI = i;
            sD = d;
        }

        public void setDirectionParams(double p, double i, double d){
            dP = p;
            dI = i;
            dD = d;
        }

    }

    public static class Correction{
        public double speed;
        public double direction;

        private Correction(double speed, double direction){
            this.speed = speed;
            this.direction = direction;
        }
    }
}
