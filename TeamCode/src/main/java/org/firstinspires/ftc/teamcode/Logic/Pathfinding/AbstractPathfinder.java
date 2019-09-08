package org.firstinspires.ftc.teamcode.Logic.Pathfinding;

import org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord.Line;
import org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord.Point;
import org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord.Polygon;

public abstract class AbstractPathfinder {
    public Field env;

    public abstract Path findQuickestPath(Point p);

    /**
     * TODO: This is close to the least efficient solution. Make it more efficient.
     * Checks if Path P is valid.
     * */
    public boolean isValidPath(Path p){
        for (Line l: p.lines) {
            for (Polygon poly : env.stuff){
                for (Point pp: env.me.vertices){
                    Line newline = l;
                    newline.moveTo(pp);
                    if (poly.intersections(newline) != null){
                        return false;
                    }
                }
                if (poly.intersections(l) != null){
                    return false;
                }
            }
        }
        return true;
    }
}
