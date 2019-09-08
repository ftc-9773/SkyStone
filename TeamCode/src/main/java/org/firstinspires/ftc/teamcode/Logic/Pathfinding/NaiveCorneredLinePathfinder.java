package org.firstinspires.ftc.teamcode.Logic.Pathfinding;

import org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord.Line;
import org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord.Point;
import org.firstinspires.ftc.teamcode.Logic.Pathfinding.Coord.Polygon;
import org.firstinspires.ftc.teamcode.Utilities.Interrupters.Interrupter;

import java.util.ArrayList;

public class NaiveCorneredLinePathfinder extends AbstractPathfinder {
    Interrupter i;
    private ArrayList<Line> uncheckedLines;

    public NaiveCorneredLinePathfinder(Field env, Interrupter i){
        this.env = env;
        this.i = i;
    }


    @Override
    public Path findQuickestPath(Point targetPoint) {
        Point startPoint = env.me.center;
        Line line = new Line(startPoint, targetPoint);
        Path guess = new Path(line);
        uncheckedLines = guess.lines;
        while(!isValidPath(guess) && i.isNotInterrupted()){
            guess = updateGuess(guess);
        }
        return null;
    }

    /**
     * Generates a new path, that is as close as possible to path p
     * */
    private Path updateGuess(Path p){
        Point point = null;
        Polygon polygon = null;
        int k = -1;
        for(int i = 0; i < p.lines.size(); i++){
            Line l = p.lines.get(i);
            for (Polygon thing : env.stuff){
                try{
                point = thing.intersections(l).get(0);
                k = i;
                polygon = thing;
                break;
                } catch (Exception e){} //A flurry of exceptions are (probably) normal
            }
            if (point != null){
                break;
            }
        }

        if (k < 0){   //Just pretend the problems don't exist.
            return null;
        }

        Point targetVert = nearestVertex(polygon.vertices, point); //Probably won't cause variable not initialised problems
        Line l0;
        Line l1;



        return null;
    }
    private ArrayList<Path> genAllPaths(Path p){
        double w = env.me.width / 2;
        double h = env.me.hight / 2;

        ArrayList<Path> paths = new ArrayList<>();

        return paths;
    }

    /**
     * Find the nearest point in a list to a given point.
     * */
    private Point nearestVertex(ArrayList<Point> l, Point inter){
        Point currLowest = l.get(0);
        double lowdist = inter.distTo(currLowest);
        double newlow = 0;
        for (Point p: l){
            newlow = p.distTo(inter);
            if (newlow < lowdist){
                currLowest = p;
                lowdist = newlow;
            }
        }
        return currLowest;
    }

}
