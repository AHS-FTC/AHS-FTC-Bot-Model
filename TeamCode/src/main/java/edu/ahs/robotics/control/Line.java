package edu.ahs.robotics.control;

/**
 * Line in standard form to be compatible with basic matrix operations.
 * </br>
 * Represents a line in 2d space without falling into the trap of functionality.
 * @author Alex Appleby
 */
public class Line {
    //ax + by = c, standard form
    public double a,b,c;

    /**
     * Creates a nonfunctional line given two points.
     * </br>
     * <a href = "https://docs.google.com/document/d/1wora-2uUPk6h0ZBJ9shOHsPu_lHaT1yhZStG9KIJYiI/edit?usp=sharing">Math</>
     * @param p1 A point on the line
     * @param p2 Another point on the line
     */
    public Line (Point p1, Point p2) {
        a = p1.x - p2.x;
        b = p2.y - p1.y;
        c = a*(p1.x) + b*(p1.y);
    }

    public Line (double a, double b, double c){
        this.a = a;
        this.b = b;
        this.c = c;
    }

    /**
     * Gets a line perpendicular to this line that runs through a point
     * @param p A point lying on the perpendicular line
     * @return The line
     */
    private Line getPerpLineAtPoint(Point p){
        double perpA = -b;
        double perpB = a;
        double perpC = (perpA * p.x) + (perpB * p.y);

        return new Line(perpA, perpB, perpC);
    }

    /**
     * Finds the intersection between two nonfunctional lines
     * @param l The line intersecting this one
     * @return
     */
    public Point findIntersection(Line l){
        
        return null;
    }

//        private Point findIntersection (Line l) {
//            double x = (b-l.b)/(l.m-m);
//            double y = getY(x);
//            return new Point(x,y);
//        }

}
