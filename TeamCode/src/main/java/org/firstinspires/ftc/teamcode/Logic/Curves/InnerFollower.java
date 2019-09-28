package org.firstinspires.ftc.teamcode.Logic.Curves;

public class InnerFollower {

    double tol = 1;
    double target = 0.5;
    double finalTheta;

    double r;
    double h;
    double k;

    double targetX;
    double targetY;

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

        double radius = altitude + other_altitude;
        r = radius;

        ///Equation of the circle: radius * radius = (x - rx) ^ 2 + (y - ry) ^ 2
        h = rx;
        k = ry;

        targetX = targetY = 0;
        finalTheta = Math.acos((-(x2 * x2 + y2 * y2) + 2 * r * r) / (2 * r * r));
    }

    //Returns array [xPower, yPower]
    double[] getXY(double x, double y){
        if (x - targetX < tol && y - targetY < tol){
            double theta;
            theta = ((-(x * x + y * y) + 2 * r * r) / (2 * r * r));
            theta = Math.acos(theta);
            double targetTheta = (finalTheta - theta) * target + theta;

            targetX = r * Math.cos(targetTheta);
            targetY = r * Math.sin(targetTheta);
        }
        double[] out = normalise(targetX - x, targetY - y);
        return out;
    }


    double[] normalise(double a, double b){
        double []out = new double[2];
        out[0] = a / (a + b);
        out[1] = b / (a + b);
        return out;
    }
}
