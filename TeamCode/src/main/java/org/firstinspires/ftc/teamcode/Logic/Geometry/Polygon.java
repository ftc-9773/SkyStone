package org.firstinspires.ftc.teamcode.Logic.Geometry;

import org.firstinspires.ftc.teamcode.Logic.pathing.Point;

import java.util.ArrayList;
import java.util.List;

/**
 * Simple Polygon implementation
 * @author Jonathan Zhou and Cadence Weddle
 * @see <a href="http://geomalgorithms.com/a03-_inclusion.html">http://geomalgorithms.com/a03-_inclusion.html</a>
 */
public class Polygon {

    private ArrayList<Point> Vertices;
    private boolean isPoint = false; //Contains only one vertex, and thus is equivalent to a point
    public String tag;

    /**
     * @author Cadence Weddle
     * @param Vertices Vertices of a simple polygon,
     *                 *Note that point order matters
     *                 *Note that only simple polygons may be constructed
     *                 *Note that polygon vertices must be in an array of v[n+1] where v[n+1]=n[0]
     *                  where n is the number of vertices.
     * @param tag Polygon identifer
     */
    public Polygon(ArrayList<Point> Vertices, String tag){
        // Constructor from explicit list of points

        this.Vertices = Vertices;
        this.tag = tag;
        if (this.Vertices.size() == 1 || (this.Vertices.size() == 2 && this.Vertices.get(0) == this.Vertices.get(1))){
            this.isPoint=true;
        }
        if (this.Vertices.get(0) != this.Vertices.get(this.Vertices.size() - 1)) {
            this.Vertices.add(this.Vertices.get(0));
        }
    }
    public Polygon(String tag, Point ... inVertices){
        //Constructor from implicit list of points
        this.tag = tag;
        for (int i=0;i<inVertices.length;i++) {
            this.Vertices.add(inVertices[i]);
        }

        if (this.Vertices.size() == 1 || (this.Vertices.size() == 2 && this.Vertices.get(0) == this.Vertices.get(1))){
            this.isPoint=true;
        }
        if (this.Vertices.get(0) != this.Vertices.get(this.Vertices.size() - 1)) {
            this.Vertices.add(this.Vertices.get(0));
        }

    }
    public boolean isIn(Polygon poly){
        //Test whether the input polygon intersects with this polygon
        //If the current polygon is a point, use the other polygons test method
        if(this.isPoint){return poly.isIn(this.Vertices.get(0));}
        return false;
    }

    /**
     * Function for testing if point is left of a line
     * @author Jonathan Zhou
     * @param LP0 first point defining input line
     * @param LP1 second point defining input line
     * @param TP  Test Point
     * @return A double:
     *           * n>0 if P2 is left of line defined by LP0 and LP1
     *           * n=0 for P2 on the line defined by LP0 and LP1
     *			 * n<\0 for P2 right of line defined by LP0 and LP1
     */
    private static double isLeft(Point LP0, Point LP1, Point TP){
        return ((LP1.xCord-LP0.xCord)*(TP.yCord-LP0.yCord) - (TP.xCord-LP0.xCord)*(LP1.yCord-LP0.yCord));
    }

    /**
     * Tests if point is in polygon using the winding number of the point
     * @author Jonathan Zhou
     * @see {@link #isLeft(Point,Point,Point) isLeft}
     * @see <a href="http://geomalgorithms.com/a03-_inclusion.html">http://geomalgorithms.com/a03-_inclusion.html</a>
     * @param TestPoint, point to test if in polygon
     * @return True if point is in polygon, else false.
     */
    public boolean isIn(Point TestPoint){
        //See above method, except for a point
        int wn = 0; //Winding number starts at 0 (obviously)
        int cycleLength = this.Vertices.size(); //Vertices follow modular arithmetic
        // Iterate across all edges of polygon
        for (int i = 0; i < cycleLength; i++) {
            if (this.Vertices.get(i).yCord <= TestPoint.yCord) { // determine crossing type
                if (this.Vertices.get((i + 1)%cycleLength).yCord > TestPoint.yCord) // test for an upward crossing
                    if (isLeft(this.Vertices.get(i), this.Vertices.get((i + 1)%cycleLength), TestPoint) > 0) // P left of edge
                        ++wn; //has a valid up intersect
            } else {
                if (this.Vertices.get((i + 1)%cycleLength).yCord <= TestPoint.yCord) // test for a downward crossing
                    if (isLeft(this.Vertices.get(i), this.Vertices.get((i + 1)%cycleLength), TestPoint) < 0) // P right of edge
                        --wn; //has a valid down intersect
            }
        }
        return !(wn==0);
    }

    public boolean isIn(double xCord,double yCord){
        Point testPoint = new Point(xCord,yCord);
        return this.isIn(testPoint);
    }

    public void translate(Vector moveVector){
        for (Point vertex: this.Vertices) vertex.move(moveVector);
    }


    public List getValues(){
        ArrayList<Double> output = new ArrayList<>();
        output.add(this.Vertices.get(0).xCord);
        output.add(this.Vertices.get(0).yCord);
        return output;
    }

    public String toString(){
        String outString = "-------\n"+this.Vertices.size()+"-gon:\n  -Tag: "+this.tag+"  \n  -Points:\n";
        for (Point vertex: this.Vertices) outString+=("    *("+vertex.xCord+", "+vertex.yCord+")\n");
        outString += "-------\n";
        return outString;
    }

    public double getX(){return this.Vertices.get(0).xCord;}

    public double getY(){return this.Vertices.get(0).yCord;}

    public void rotDeg(double angle, Point Point){
        //Do stuff
    }

    /**
     *
     */
    public void rotRad(double angle, Point Point){
        //Do stuff
    }


}
