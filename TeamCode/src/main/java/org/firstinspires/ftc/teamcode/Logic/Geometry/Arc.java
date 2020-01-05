package org.firstinspires.ftc.teamcode.Logic.Geometry;

import org.firstinspires.ftc.teamcode.Logic.pathing.Point;

public class Arc {
    final static double TAU = Math.PI*2;
    Point start;
    Point end;
    double heading;
    public boolean isStraight = false;
    public double radius;
    public double theta;

    public Arc(Point start, Point end, double heading){
        this.start = start;
        this.end = end;
        this.heading = heading;
        if (Math.abs(heading) == 0) {
            isStraight = true;
            radius = Double.POSITIVE_INFINITY;
        }
        else{
            double deltaX = end.xCord -start.xCord;
            double deltaY = end.yCord -start.yCord;
            theta = Math.atan2(deltaY,deltaX);
            radius = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)) / (2 * Math.sin(heading- theta));
        }
    }
    public Point getCenter(){
        if(isStraight) return null;
        Point center;
        if(heading % TAU > theta % TAU){
             center = new Point(0,0);
            center.xCord = start.xCord + Math.cos(heading- Math.PI*0.5)*radius;
            center.yCord = start.yCord + Math.sin(heading- Math.PI*0.5)*radius;
        } else {
             center = new Point(0,0);
            center.xCord = start.xCord + Math.cos(heading + Math.PI*0.5)*radius;
            center.yCord = start.yCord + Math.sin(heading + Math.PI*0.5)*radius;
        }
        return center;
    }
    public double getRadius(){
        return radius;
    }
    public Point getStartPoint(){
        return start;
    }
    public Point getEndPoint(){
        return end;
    }
    public double getLength(){
        if(isStraight)
        return radius * 2 * (heading - theta);
        return radius * TAU * heading / (Math.PI / 2);
    }
}
