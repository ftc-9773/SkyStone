package org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord;


/**
 * Atomic geometry class.
 * Just useful as a container, and has some utility functions
 *
 * You can directly access x, y, z, r, and t, but use their setter functions to make sure everything is updated.
 * (t is theta, and for polar form)
 *
 * Functions are:
 *    Point(x, y, z)
 *    Point(x, y, true)
 *    Point() {Set at (0, 0, 0)}
 *    Point(r, t, false) //Create using polar mode
 *    isEqual(Point p) //Checks whether point p has identical coordinates
 *    isClose(Point p, double tol) //Checks whether point p is within tol of this point, and if the z coordinates are identical
 *    setX(x)
 *    setY(y)
 *    setZ(z)
 *    setR(r)
 *    setTheta(t)
 * */
public class Point {
    public double x;
    public double y;
    public double z; // Not guaranteed to exist. Usage of z coordinate should probably be like enums, probably.

    public double t;
    public double r;

    public Point(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;

        this.r = Math.sqrt(x * x + y * y);
        this.t = Math.atan2(y, x);
    }

    public Point(double x, double y, boolean cart){
        if (cart){
            this.x = x;
            this.y = y;
            this.z = z;

            this.r = Math.sqrt(x * x + y * y);
            this.t = Math.atan2(y, x);
        }
        else{
            this.r = x;
            this.t = y;
            this.z = 0;

            this.x = Math.cos(t) * r;
            this.y = Math.sin(t) * r;
        }
    }

    public Point(){
        this(0, 0, 0);
    }

    public boolean isEqual(Point p){
        return this.x == p.x && this.y == p.y && p.z == this.z;
    }

    /**
     * Checks whether the point p is within tol distance from this point. Requires z coordinates be equal.
     * */
    public boolean isClose(Point p, double tol){
        return Math.sqrt(Math.pow(Math.abs(this.x - p.x), 2) + Math.pow(Math.abs(this.y - p.y), 2)) <= tol && p.z == this.z ;
    }

    /**
     * Set x, recalculate r and t
     * */
    public void setX(double x){
        this.x = x;
        this.r = Math.sqrt(x * x + y * y);
        this.t = Math.atan2(y, x);
    }

    /**
     * Set y, recalculate r and t
     * */
    public void setY(double y) {
        this.y = y;
        this.r = Math.sqrt(x * x + y * x);
        this.t = Math.atan2(y, x);
    }

    /**
     * Set Z, recalculate nothing
     * */
    public void setZ(double z){
        this.z = z;
    }

    public void setTheta(double t){
        this.t = t;
        this.x = Math.cos(t) * r;
        this.y = Math.sin(t) * r;
    }

    public void setR(double r){
        this.r = r;
        this.x = Math.cos(t) * r;
        this.y = Math.sin(t) * r;
    }

    public void rotatePoint(Point p, double t){
        Point originPoint = new Point(x - p.x, y - p.y, true);
        originPoint.setTheta(originPoint.t + t);
        originPoint.setX(originPoint.x + p.x);
        originPoint.setY(originPoint.y + p.y);
        setX(originPoint.x);
        setY(originPoint.y);
    }

    public double distTo(Point p){
        return Math.sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));
    }
}

