package org.firstinspires.ftc.teamcode.Logic.Pathfinding;


import static java.lang.Math.*;

public class Point {
    double x;
    double y;


    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    //Express coordinates relative to some point (x, y)
    public Point relative(double x, double y){
        return new Point(x - this.x, y - this.y);
    }
    //reverse this.relative(x, y)
    public Point global(double x, double y){
        return new Point(x + this.x, y + this.y);
    }

    public double dist(double x, double y){
        return sqrt(pow(this.x - x, 2) + pow(this.y -y, 2));
    }

    public double dist(Point point){
        return this.dist(point.x, point.y);
    }

    public void from(Point point){
        this.x = point.x;
        this.y = point.y;
    }
}
