package org.firstinspires.ftc.teamcode.Logic.Curves;

import android.util.Log;

public class InnerFollower {
    private static final boolean DEBUG = false;

    double tol = 1;
    double target = 0.5;
    double finalTheta;

    public double r;
    public double h;
    public double k;

    double targetX;
    double targetY;

    double finalX, finalY;

    double xPow;
    double yPow;

    public InnerFollower(double x1, double y1, double x2, double y2, double theta){
        double rx, ry, mx, my;
        mx = (x1 + x2) / 2;
        my = (y1 + y2) / 2;

        theta = theta * Math.PI / 180;

        double len_to_midpoint = Math.sqrt(mx * mx + my * my);
        double altitude = Math.tan(theta) * len_to_midpoint;
        double other_altitude = len_to_midpoint * len_to_midpoint / altitude;

        rx = mx + Math.sin(theta) * other_altitude;
        ry = my - Math.cos(theta) * other_altitude;

        double radius;
        radius  = Math.sqrt(rx * rx + ry * ry);
        r = radius;
        if (DEBUG) Log.d("INNERFOLLOW", "Radius " + r);
        ///Equation of the circle: radius * radius = (x - rx) ^ 2 + (y - ry) ^ 2
        h = rx;
        k = ry;

        targetX = targetY = 0;
        finalTheta = Math.acos(-((x2 * x2 + y2 * y2 - 2 * r * r) / (2 * r * r)));
        finalX = x2;
        finalY = y2;
    }

    //Returns array [xPower, yPower]
    public double[] getXY(double x, double y){
        if (Math.abs(x - targetX) < tol && Math.abs(y - targetY) < tol){
            double theta;
            theta = -(((x * x + y * y - 2 * r * r)) / (2 * r * r));
            theta = Math.acos(theta);
            double targetTheta = (finalTheta - theta) * target + theta;
            if (DEBUG) Log.d("INNERFOLLOW", "Theta: " + theta);
            targetX = r * Math.cos(targetTheta);
            targetY = r * Math.sin(targetTheta);
            double[] out = new double[2];
            out[0] = 0;
            out[1] = 0;
            if (DEBUG)Log.d("INNERFOLLOW", "xp " + out[0] + " yp" + out[1]);
            return out;
        }
        if (finalX - x < tol && finalY - y < tol){
            double[] out = new double[2];
            out[0] = 0;
            out[1] = 0;
            return out;
        }
        if (DEBUG) Log.d("INNERFOLLOW", "" + targetX + " : " + targetY + " : " + x + " : " + y);
        double[] out = normalise(targetX - x, targetY - y);
        if (DEBUG) Log.d("INNERFOLLOW", "xp " + out[0] + " yp" + out[1]);
        return out;
    }


    double[] normalise(double a, double b){
        double []out = new double[2];
        out[0] = a / Math.sqrt(a * a + b * b);
        out[1] = b / Math.sqrt(a * a + b * b);
        return out;
    }
}
