package edu.ahs.robotics.control;

public class Vector {
    public double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Makes a unit vector for a given heading.
     * @param direction Standard cartesian angle in radians.
     * @return A vector of length 1 in the specified direction.
     */
    public static Vector makeUnitVector(double direction){
        double x = Math.cos(direction);
        double y = Math.sin(direction);
        return new Vector(x,y);
    }

    public Vector getPerpVector(){
        return new Vector(-y,x);
    }

    /**
     * Returns a 1 or -1 multiplier depending on if a point is on the left or right side of a vector.
     * Note: Positive value means left of line. This may contradict older code.
     * @param botPosition where the robot is, effectively an offset for the path vector.
     */
    public int sideOfLine(Point p, Point botPosition){
        Point pAfterShift = new Point(p.x - botPosition.x, p.y - botPosition.y);

        Vector perp = getPerpVector();
        Vector pVector = new Vector(pAfterShift.x,pAfterShift.y);

        double dp = pVector.dotProduct(perp);

        return (int)Math.signum(dp);
    }

    public double dotProduct(Vector v){
        return (x * v.x) + (y * v.y);
    }

    /**
     * Inverts this vector, effectively multiplying by -1, retaining magnitude but flipping direction.
     * @return This instance, for chaining.
     */
    public Vector invert(){
        y = -y;
        x = -x;
        return this;
    }
    /**
     * Scales this vector by a constant, retaining direction but scaling magnitude.
     * @param scale The scalar constant. Can be negative.
     * @return This instance, for chaining.
     */
    public Vector scale(double scale){
        y = y * scale;
        x = x * scale;
        return this;
    }
}
