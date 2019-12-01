package org.firstinspires.ftc.teamcode.Logic.Pathfinding;

import org.firstinspires.ftc.teamcode.HardwareControl.Robot;

import java.util.ArrayList;
import static java.lang.Math.*;

/**
 * HOW TO USE THIS CLASS:
 *
 * OPMODE:
 * (once) calls setWaypoints(ArrayList<Points>)
 *
 * (every updateLoop) calls getMovementVector(double x, double y)
 *
 *
 * getMovementVector() - returns [xpower, ypower, turnpower] // TODO implement turnpower //TODO Rewrite entire codebase in an easier way.
 *  - calls Point getFollowPoint(state) checks which point it should be following
 *  - calls getVector(currentPoint, followPoint) to get the movement vector from the follow point
 *  - returns vector
 *
 * getFollowPoint() - returns point to follow:
 *  - calls getClosestLine() to
 *  - - check which waypoints we're between
 *  - - calculate all lines between them that are within radius r.
 *  - - Get the one with the largest index.
 *  - -
 *  - then calls getLineCircleIntersection(radius)
 * */

public class PurePursuit {
    ArrayList<Point> waypoints;
    Robot robot;
    double r; //Radius to follow

    /**
     * Geometries for easier codings
     * */
    private class Line {
        Point p1;
        Point p2;
        double m;
        double b;
        public Line(Point p1, Point p2){
            this.p1 = p1;
            this.p2 = p2;

            if (abs(p1.y - p2.y) < 0.001){
                p2.y = p1.y > p2.y ? p1.y: p2.y  + 0.001;
            }
            if (abs(p1.x - p2.x) < 0.001){
                p2.x = p1.x > p2.x ? p1.x: p2.x  + 0.001;
            }
            m = (p1.y - p2.y) / (p1.x - p2.x);
            b = p1.y - p1.x * m;

        }
    }

    private class IndexPoint extends Point{
        int i;
        public IndexPoint(Point p, int index){
            super(0,0);
            this.from(p);
            this.i = index;
        }
        public int index(){
            return i;
        }
    }

    //Waypoints can't be empty
    public PurePursuit(ArrayList<Point> waypoints, Robot robot, double r){
        waypoints.add(0, new Point(robot.x, robot.y));
        this.waypoints = waypoints;
        int i = 0;
        for (Point p: waypoints) {
            waypoints.set(i, new IndexPoint(waypoints.get(i), i));
        }
        this.robot = robot;
        this.r = r;
    }

    public double[] getMovementVector(){
        Point followPoint = getFollowPoint();
        double []moveVector = new double[2];
        double mag = followPoint.dist(0,0);

        moveVector[0] = followPoint.x / mag;
        moveVector[1] = followPoint.y / mag;

        return moveVector;
    }

    private Point getFollowPoint(){
        Point robotLoc = new Point(robot.x, robot.y);
        Line closestLine = getClosestLine(robotLoc);
        ArrayList<Point> intersects = getCircleLineIntersections(robotLoc, this.r, closestLine);
        Point followPoint;
        if (intersects.size() == 1){
            followPoint = intersects.get(0);
        } else {
            followPoint = intersects.get(0).dist(closestLine.p2) < intersects.get(1).dist(closestLine.p2) ? intersects.get(0) : intersects.get(1);
        }
        return followPoint;
    }

    //Get's a line at most distance r from the robot that is closest to the target final position.
    private Line getClosestLine(Point point){
        double dist = 1000000000; //Infinity
        IndexPoint closestPoint = null;
        int i = 0;
        ArrayList<IndexPoint> possibleStartPoints = new ArrayList<>();
        for (Point p : waypoints.subList(0,waypoints.size() - 1)) {
            if (distToLine(point, new Line(p,waypoints.get(i + 1))) < r){
                possibleStartPoints.add(new IndexPoint(p, i));
                i += 1;
            }
        }
        closestPoint = possibleStartPoints.get(possibleStartPoints.size() -1 );
        return new Line(closestPoint, waypoints.get(closestPoint.index() + 1));
    }

    private double distToLine(Point p, Line l){
        double _m = -1 * l.m;
        double _b = p.y - p.x * _m;
        double _x = (l.b - _b) / (_m - l.m);
        double _y = _x * _m + _b;
        return p.dist(_x, _y);
    }

    private ArrayList<Point> getCircleLineIntersections(Point circleCenter, double r, Line l){
        ArrayList<Point> intersects = new ArrayList<>();
        Point adjustL1 = l.p1.relative(circleCenter.x, circleCenter.y);
        Point adjustL2 = l.p2.relative(circleCenter.x, circleCenter.y);
        Line l_adust = new Line(adjustL1, adjustL2);

        double qa = (pow(l_adust.m, 2) + 1);
        double qb = (2 * l_adust.m * l_adust.b);
        double qc = (l_adust.b * l_adust.b - r * r);
        double d = (pow(qb, 2) - 4 * qa * qc);

        if (d < 0){ // No intersects found
            return null;
        }

        intersects.add(new Point((-qb + pow(d, 0.5)) / 2 / qa, ((-qb + pow(d, 0.5)) / 2 / qa) * l_adust.m + l_adust.b).global(circleCenter.x, circleCenter.y));
        intersects.add(new Point((-qb - pow(d, 0.5)) / 2 / qa, ((-qb - pow(d, 0.5)) / 2 / qa) * l_adust.m + l_adust.b).global(circleCenter.x, circleCenter.y));
        return intersects;
    }
}
