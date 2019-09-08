package org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord;

/**
 * Atomic Geometric class
 * Contains the endpoints, and some utility functions
 * */
public class Line {
    public Point p1;
    public Point p2;
    public double length;
    public double m;
    public double b;

    public double xMin;
    public double xMax;
    public double yMin;
    public double yMax;

    /**
     * @param p11 first endpoint
     * @param p22 second endpoint
     * */
    public Line(Point p11, Point p22){
        p1 = p11;
        p2 = p22;
        length = Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
        try {
            m = (p1.y - p2.y) / (p1.x - p2.x);
            b = p1.y - p1.x * m;
        } catch (Exception e) {
            m = Double.POSITIVE_INFINITY;
            b = 0;
        }
        xMin = Math.min(p1.x, p2.x);
        xMax = Math.max(p1.x, p2.x);
        yMin = Math.min(p1.y, p2.y);
        yMax = Math.max(p1.y, p2.y);
    }

    /**
     * Returns the point of intersection between two lines, or null if they don't intersect.
     * @param l the line to check against this.
     * */
    public Point intersectPoint(Line l){
        double intersectX = (b - l.b) / (l.m - m);
        if (contains(intersectX, true)){
            return new Point(intersectX, m * intersectX + b, true);
        }
        return null;
    }

    /**
     * Checks whether the line intersects this line
     * @param l the line to check against this
     * */
    public boolean intersects(Line l){
        return intersectPoint(l) != null;
    }

    /**
     * Checks whether the line is parallel to this line
     * @param l the line to check against this
     * */
    public boolean isParallel(Line l){
        return m == l.m;
    }

    /**
     * Checks whether the line is normal to this line
     * @param l the line to check against this
     * */
    public boolean isNormal(Line l){
        return m == -1 / l.m;
    }

    /**
     * Checks whether the a certain x or y value is in between the endpoints of the line
     * @param v The value to check
     * @param isX Whether to treat v as the x value or y value.
     * */
    public boolean contains(double v, boolean isX){
        return (isX ? xMin : yMin) <= v && v <= (isX ? yMin: yMax);
    }

    /**
     * Move the line a distance
     * @param dx the distance to move in the x direction
     * @param dy the distance to move in the y direction
     * */
    public void move(double dx, double dy){
        this.p1.setX(p1.x + dx);
        this.p1.setY(p1.y + dy);
        this.p2.setX(p2.x + dx);
        this.p2.setY(p2.y + dy);

        try {
            m = (p1.y - p2.y) / (p1.x - p2.x);
            b = p1.y - p1.x * m;
        } catch (Exception e) {
            m = Double.POSITIVE_INFINITY;
            b = 0;
        }
        xMin = Math.min(p1.x, p2.x);
        xMax = Math.max(p1.x, p2.x);
        yMin = Math.min(p1.y, p2.y);
        yMax = Math.max(p1.y, p2.y);
    }

    /**
     * Move the first endpoint to point p, and keep the length and slope the same.
     * @param p target point of the first endpoint
     * */
    public void moveTo(Point p){
        double dx = p.x - p1.x;
        double dy = p.y - p2.y;
        move(dx, dy);
    }

    public void setP1(Point p){
        this.p1 = p;

        xMin = Math.min(p1.x, p2.x);
        xMax = Math.max(p1.x, p2.x);
        yMin = Math.min(p1.y, p2.y);
        yMax = Math.max(p1.y, p2.y);

        try {
            m = (p1.y - p2.y) / (p1.x - p2.x);
            b = p1.y - p1.x * m;
        } catch (Exception e) {
            m = Double.POSITIVE_INFINITY;
            b = 0;
        }
    }

    public void setP2(Point p){
        this.p2 = p;

        xMin = Math.min(p1.x, p2.x);
        xMax = Math.max(p1.x, p2.x);
        yMin = Math.min(p1.y, p2.y);
        yMax = Math.max(p1.y, p2.y);

        try {
            m = (p1.y - p2.y) / (p1.x - p2.x);
            b = p1.y - p1.x * m;
        } catch (Exception e) {
            m = Double.POSITIVE_INFINITY;
            b = 0;
        }

    }
}
