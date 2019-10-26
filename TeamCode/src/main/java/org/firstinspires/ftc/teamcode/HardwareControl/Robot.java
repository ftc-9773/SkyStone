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
    DistSensorArray distSensorArray;

    //Default constructer
    public Robot(){}

    public Robot(MecanumDrivebase drivebase, Gyro gyro, HardwareMap hardwareMap){
        this.drivebase = drivebase;
        this.gyro = gyro;
        distSensorArray = new DistSensorArray(hardwareMap);
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
    }

}
