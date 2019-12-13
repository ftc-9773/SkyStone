package org.firstinspires.ftc.teamcode.Logic.Pathfinding;

public class Point {
    //State of the robot.
    //X coordinate, Y coordinate, and heading of the robot, in absolute coordinates.
    double x;
    double y;
    double h;

    public Point(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public Point rel(double x, double y) {
        return rel(x, y, 0);
    }

    public Point rel(double x, double y, double h) {
        return new Point(this.x - x, this.y - y, this.h - h);
    }

    public void from(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.h = h;
    }

}
