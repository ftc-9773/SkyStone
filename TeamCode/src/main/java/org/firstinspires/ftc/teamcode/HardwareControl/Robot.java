package org.firstinspires.ftc.teamcode.HardwareControl;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;

/**
 * Base class for robots. Classes for each individual robot should extend this class.
 *
 * */
public class Robot {
    public MecanumDrivebase drivebase;
    public Gyro gyro;
    public double heading;
    public double x;
    public double y;

    private long lastx[], lasty[];

    DistSensorArray distSensorArray;

    //Default constructer
    public Robot(){}

    public Robot(MecanumDrivebase drivebase, Gyro gyro, HardwareMap hardwareMap){
        this.drivebase = drivebase;
        this.gyro = gyro;
        distSensorArray = new DistSensorArray(hardwareMap);
        lastx = lasty = drivebase.getMotorPositions();
    }

    public double getHeading(){
        return gyro.getHeading();
    }

    public double[] getDistSensorReadings(){
        double[] out = new double[2];
        out[0] = distSensorArray.rReading();
        out[1] = distSensorArray.lReading();
        return out;
    }

    public void update(){
        drivebase.update();
        heading = getHeading();
        getdistancetraveledencoder();
    }

    private void getdistancetraveledencoder(){
        long cur[] = drivebase.getMotorPositions();
        double dx = 0;
        dx += ((cur[0] - lastx[0]) + (cur[1] - lasty[1])) / 2;
        x += dx;
        long cury[] = drivebase.getMotorPositions();
        double dy = 0;
        dy += ((cury[0] - lasty[0]) - (cury[1] - lasty[1])) / 2;
        y += dy;

        lastx = cur;
        lasty = cury;
    }

}
