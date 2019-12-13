package org.firstinspires.ftc.teamcode.Logic.Pathfinding;

import org.firstinspires.ftc.teamcode.HardwareControl.Robot;

/**
 * Follows a path using the pure pursuit algorithem
 */
public class PathFollower {
    final String TAG = "PathFollower";
    Path path;
    Point robot_loc;
    Point followPoint;

    Robot robot;
    double optimal_orientation; //NOT YET IMPLEMENTED. TODO See if strafing is slower than driving straight.
    double r; //Radius of the circle to draw around the robot.

    public double xp, yp, hp;
    boolean active = false;

    public PathFollower(Robot robot, double r){
        this.robot = robot;
        this.robot_loc = new Point(robot.x, robot.y, robot.heading);
        this.r = r;
    }

    /**
     * Finds followPoint and recalculates xp, yp, and hp.
     * also updates path if a segment is completed.
     * */
    public void getPower(){
        robot_loc.from(robot.x, robot.y, robot.heading);
        for (Line l: path.queue) {

        }
    }

    public void done(){
        active = false;
    }
    public void setPath(Path p){
        path = p;
    }

    public void start() {
        active = true;
    }


    public void update(){
        if (active){
            getPower();
            robot.drivebase.drive(xp, yp, hp, false);
        }
    }
}
