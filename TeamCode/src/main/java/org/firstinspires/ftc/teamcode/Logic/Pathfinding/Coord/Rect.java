package org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord;

import java.security.InvalidParameterException;
import java.util.ArrayList;

public class Rect extends Polygon {
    public double width;
    public double hight;
    public double diag;

    public Rect(ArrayList<Point> ov){
        super(ov);
        if (ov.size() != 4){
            throw new ExceptionInInitializerError("Rectangle constructor has more or less than 4 vertices.");
        }
        width = vertices.get(0).distTo(vertices.get(1));
        hight = vertices.get(2).distTo(vertices.get(3));
        diag = vertices.get(0).distTo(vertices.get(2));
    }

}
