package org.firstinspires.ftc.teamcode.Logic.Pathfinding;

import java.util.ArrayList;

public class Path {
    ArrayList<Line> queue = new ArrayList<>();



    public Path(){

    }
    public void addSegment(Line l){
        queue.add(l);
    }

    public void next(){
        queue.remove(0);
    }
}
