package org.firstinspires.ftc.teamcode.Logic.Pathfinding;

/**
 * Ordered set of two points.
 * */
public class Line {
    Point start;
    Point end;

    Point rel_start = null;
    Point rel_end = null;
    double m;
    double b;
    double rel_b;
    int magic_x;
    int magic_y;

    public Line(Point p1, Point p2){
        start = p1;
        end = p2;

        if (Math.abs(start.x - end.x) < 0.001){
            m = (start.y - end.y) / (start.x - end.x + 0.001);
        } else {
            m = (start.y - end.y) / (start.x - end.x);
        }
        b = start.y - start.x * m;
        magic_x = start.x <= end.x ? 1 : -1;
        magic_y = start.y <= end.y ? 1 : -1;
    }

    public void rel(double x, double y){
        rel_start = start.rel(x, y);
        rel_end = end.rel(x, y);
        rel_b = rel_start.y - rel_start.x * m;
    }

    public double dist(Point p){
        //Calculate intersection point:
        double eng_1_m = -1 / m;

        return 0;
    }

}
