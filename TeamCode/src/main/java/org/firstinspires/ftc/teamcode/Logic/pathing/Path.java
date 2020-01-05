package org.firstinspires.ftc.teamcode.Logic.pathing;

import java.util.ArrayList;

public class Path {
    ArrayList<Point> waypoints = new ArrayList();

    public Path(){

    }

    public Path(Point point){
        waypoints.add(point);
    }
}
