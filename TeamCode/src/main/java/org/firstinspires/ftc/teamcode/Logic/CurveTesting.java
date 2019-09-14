package org.firstinspires.ftc.teamcode.Logic;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;

public class CurveTesting {

    public MecanumDrivebase drivebase;
    public Gyro gyro;

    double k;
    double h;

    public CurveTesting(HardwareMap hwmp, Telemetry telemetry){
    }

    /**
     * rx, ry = coordinates of center of circle
     *
     * */
    public void driveinCurve(double x1, double y1, double x2, double y2, double theta){
        double rx, ry, mx, my;
        mx = (x1 + x2) / 2;
        my = (y1 + y2) / 2;

        double len_to_midpoint = Math.sqrt(mx * mx + my * my);
        double altitude = Math.tan(theta) * len_to_midpoint;
        double other_altitude = len_to_midpoint * len_to_midpoint / altitude;

        rx = mx + Math.sin(theta) * other_altitude;
        ry = my - Math.cos(theta) * other_altitude;

        double radius = altitude + other_altitude;

        ///Equation of the circle: radius * radius = (x - rx) ^ 2 + (y - ry) ^ 2
        //So then take the derivitive with respect to both x and y.

        h = rx;
        k = ry;

    }

    public double y_prime(double x, double y){
        return (2 * h - 2 * x) / (2 * y - 2 * k);
    }
    public double x_prime(double x, double y){
        return (2 * k - 2 * y) / (2 * x - 2 * h);
    }
}

