package org.firstinspires.ftc.teamcode.Logic.Geometry;

import org.firstinspires.ftc.teamcode.Logic.pathing.Point;
import org.firstinspires.ftc.teamcode.Utilities.misc.MathOps;

/**
 * Implements a vector
 * @author Nicky
 */
public class Vector {

    private double xComponent = 0;
    private double yComponent = 0;

    /**
     * Converts an angle to the smallest equivalent angle
     * Can be done without a loop---Please fix. Jonathan has done so
     * @param angle an angle in radians
     * @return an angle in radians
     * */
    private double modPi(double angle) {
        return MathOps.modPi(angle);
    }

    /**
     * Creates a cartesian vector of the form [x, y]
     * @param xMag X value if isCartesian is true. If polar, magnitude of vector
     * @param yAng Y value if isCartesian is true. If polar, angle in radians from 0 radians
     * @param isCartesian Tells whether the input is cartesian or polar
     * */
    public Vector(boolean isCartesian, double xMag, double yAng) {
        if (isCartesian) {
            xComponent = xMag;
            yComponent = yAng;
        } else {
            xComponent = Math.sin(yAng) * xMag;
            yComponent = Math.cos(yAng) * xMag;
        }
    }


    /**
     * Sets the values of the vector
     * @param xMag X value if isCartesian is true. If polar, magnitude of vector
     * @param yAng Y value if isCartesian is true. If polar, angle in radians from 0 radians
     * @param isCartesian Tells whether the input is cartesian or polar
     * */
    public void set(boolean isCartesian, double xMag, double yAng) {
        if (isCartesian) {
            xComponent = xMag;
            yComponent = yAng;
        } else {
            xComponent = Math.sin(yAng) * xMag;
            yComponent = Math.cos(yAng) * xMag;
        }
    }

    /**
     * Add a decomposed vector to the vector
     * @param xMag X value to be added if isCartesian is true. If polar, magnitude of vector
     * @param yAng Y value to be added if isCartesian is true. If polar, angle in radians from 0 radians
     * @param isCartesian Tells whether the input is cartesian or polar     * */
    public void addVector (boolean isCartesian, double xMag, double yAng) {
        if (isCartesian) {
            xComponent += xMag;
            yComponent += yAng;
        } else {
            xComponent += xMag * Math.sin(yAng);
            yComponent += xMag * Math.cos(yAng);
        }
    }
    /**
     * Add a vector
     * @param vector The vector to be added. All vectors are cartesian
     * */
    public void addVector(Vector vector){
        this.xComponent += vector.getX();
        this.yComponent += vector.getY();
    }


    /**
     * Rotate the vector by <code>angle</code> radians
     * @param angle angle to transform the vector
     * */
    public void rotateVector(double angle) {
        set(false, getMagnitude(), getAngle() + angle);
    }

    //Fetch values
    public double getX () { return xComponent; }
    public double getY () { return yComponent; }

    /**
     * Get the magnitude of the vector
     * @return polar magnitude of the vector
     * */
    public double getMagnitude () { return Math.sqrt( Math.pow(xComponent, 2) + Math.pow(yComponent, 2)); }

    /**
     * Gets the angle of the vector
     * @return polar angle of vector
     * */
    public double getAngle () {
        final double angle = Math.PI/2 - Math.atan2(yComponent, xComponent);

        if (angle < 0) {
            return angle + 2 * Math.PI;
        }  else {
            return angle;
        }
    }
    public Point toPoint(){
        return new Point(this.xComponent, this.yComponent);
    }
}
