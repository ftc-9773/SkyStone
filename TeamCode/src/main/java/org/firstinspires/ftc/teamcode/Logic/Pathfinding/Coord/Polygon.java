package org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord;

import java.util.ArrayList;

/**
 * Polygon class
 * Move and Rotate are not that efficient, and there may be some error because of floating point precision problems.
 * */
public class Polygon {
    public ArrayList<Line> lines;
    public ArrayList<Point> vertices;
    public Point center;

    public Polygon(ArrayList<Point> OrderedVertices){
        this.vertices = OrderedVertices;
        lines = new ArrayList<Line>();
        int len = OrderedVertices.size();
        double ybar = OrderedVertices.get(len - 1).y;
        double xbar = OrderedVertices.get(len - 1).x;
        for (int i = 0; i < len; i++){
            lines.add(new Line(OrderedVertices.get(i), OrderedVertices.get(i + 1)));
            xbar += OrderedVertices.get(i).x;
            ybar += OrderedVertices.get(i).y;
        }
        center = new Point(xbar / len, ybar / len, true);
    }

    public void move(double x, double y){
        for (Point p : vertices){
            p.setX(p.x + x);
            p.setY(p.y + y);
            int len = vertices.size();
            lines = new ArrayList<Line>();
            for (int i = 0; i < len; i++){
                lines.add(new Line(vertices.get(i), vertices.get(i + 1)));
            }
            center.setX(center.x + x);
            center.setY(center.y + y);
        }
    }

    public void rotate(double t){
        for (Point p : vertices) {
            p.rotatePoint(center, t);
        }
        int len = vertices.size();
        lines = new ArrayList<Line>();
        for (int i = 0; i < len; i++){
            lines.add(new Line(vertices.get(i), vertices.get(i + 1)));
        }
    }
    public ArrayList<Point> intersections(Line l){
        ArrayList<Point> intersects = new ArrayList<>();
        for (Line line : lines) {
            intersects.add(l.intersectPoint(line));
        }
        int size = intersects.size();
        ArrayList nulls = new ArrayList();
        for (int i = 0; i < size; i++){
            if (intersects.get(i) == null){
                nulls.add(i);
            }
        }
        size = nulls.size();
        for (int i = size; i >= 0; i--){
            intersects.remove(nulls.get(i));
        }
        return intersects;
    }

    public boolean overlaps(Polygon poly){
        for (Line l: lines){
            for (Line ll : poly.lines){
                if (l.intersects(ll))
                    return true;
            }
        }
        return false;
    }
}
