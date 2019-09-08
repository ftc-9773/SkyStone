package org.firstinspires.ftc.teamcode.Logic.Pathfinding;

import org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord.Line;

import java.util.ArrayList;
import java.util.List;

public class Path {
    public ArrayList<Line> lines;

    public Path(List<Line> l){
        lines = new ArrayList<>(l);
    }

    public Path(Line l){
        lines = new ArrayList<>();
        lines.add(l);
    }

    /**
     * Move the entire path dx, dy
     * */
    public void move(double dx, double dy){
        ArrayList newlines = new ArrayList();
        for (int i = 0; i < lines.size(); i++){
            Line l = lines.get(i);
            l.move(dx, dy);
            newlines.add(l);
        }
        this.lines = newlines;
    }
}
