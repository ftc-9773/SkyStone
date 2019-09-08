package org.firstinspires.ftc.teamcode.Logic.Geometry;

//import org.firstinspires.ftc.teamcode.Logic.Geometry.Vector;

/**
 * @author Cadence Weddle and Jonathan Zhou
 * @version 2018-5-1
 * Used for Coordinate system and Polygon
 */
public class Point{
    public double xCord, yCord;
    public Point(double xCord, double yCord){
        this.xCord = xCord;
        this.yCord = yCord;
    }
    /**
     * Moves the point in accordance with the input vector
     * @author Cadence Weddle
     * @param moveVector Vector to move by
     */
    public void move(Vector moveVector){
        this.xCord += moveVector.getX();
        this.yCord += moveVector.getY();
    }

    /**
     * Used to get info about object though an output stream
     * @author Jonathan Zhou
     * @return String indicating the ordinate and abyssca of said point (Cartesian)
     */
    public String toString(){
        return "------------\nPoint\nX:"+this.xCord+"\nY:"+this.yCord+"\n------------\n";
    }
    public boolean AreSame(Point compare, double tolerance){
        if (Math.abs(this.xCord - compare.xCord) < tolerance && Math.abs(this.yCord - compare.yCord) < tolerance){
            return true;
        }
        return false;
    }
    public Vector toVector(){
        return new Vector(true, this.xCord, this.yCord);
    }
}