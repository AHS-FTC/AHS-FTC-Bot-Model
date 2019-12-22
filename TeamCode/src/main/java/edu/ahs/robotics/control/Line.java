package edu.ahs.robotics.control;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

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
     * Finds the intersection between two nonfunctional lines.
     * Utilizes standard matrix multiplication and linear algebra.
     * <br/>
     * <a href = "https://docs.google.com/document/d/1-YaZ2nIiVRlJiNXVc9DvnZ-Pd_QOwHxHLTtGJqQMJJA/edit?folder=1EVwkBM0pBycM7-6TNpljJVHYB1fVUfgU">Math</>
     * @param l The line intersecting this one
     * @return The intersection
     */
    public Point findIntersection(Line l){
        //ax + by = c
        //dx + ey = f

        //[ a , b ]  //Matrix A
        //[ d , e ]

        double a = this.a, b = this.b;
        double d = l.a,    e =  l.b;

        double c = this.c, f = l.c;


        double det = 1/(a*e - b*d); //determinate of A

        if (det == Double.POSITIVE_INFINITY){
            throw new Warning("Tried to find the intersection of two parallel lines in the Line class");
        }

        //[aInv, bInv]  //Matrix A^-1
        //[dInv, eInv]

        double aInv = e * det, bInv = -b * det;
        double dInv = -d * det, eInv = a * det;

        //Ax = b
        //x = A^-1 * b

        //[x] = [aInv, bInv ] *  [c]
        //[y] = [dInv, eInv ] *  [f]

        double x = (aInv * c) + (bInv * f);
        double y = (dInv * c) + (eInv * f);

        return new Point(x,y);
    }

//        private Point findIntersection (Line l) {
//            double x = (b-l.b)/(l.m-m);
//            double y = getY(x);
//            return new Point(x,y);
//        }

}
