package edu.ahs.robotics.control;

/**
 * PID system for vector velocity that considers both magnitude and direction.
 * Could be refactored to share code with other PID systems.
 * @author Alex Appleby
 */
public class VelocityPID{
    private PID speedPID, directionPID;

    public VelocityPID(Config config) {
        speedPID = new PID(config.sP, config.sI, config.sD);
        directionPID = new PID(config.dP, config.dI, config.dD);
    }

    /**
     * Given current and target velocity vectors, find suitable PID adjustments.
     * @return corrections in an enclosed Correction class.
     */
    public Correction getCorrection(Velocity currentVelocity, Velocity targetVelocity){
        double speedCorrection = speedPID.getCorrection(currentVelocity.speed, targetVelocity.speed);
        double directionCorrection = directionPID.getCorrection(currentVelocity.direction, targetVelocity.direction);

        return new Correction(speedCorrection, directionCorrection);
    }

    /**
     * Manages tuning parameters for VelocityPID.
     */
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

    /**
     * Enclosed class that contains correction information returned by the getCorrection() method.
     */
    public static class Correction{
        public double speed;
        public double direction;

        private Correction(double speed, double direction){
            this.speed = speed;
            this.direction = direction;
        }
    }
}
