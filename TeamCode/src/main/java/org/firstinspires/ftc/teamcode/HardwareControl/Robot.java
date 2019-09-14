package org.firstinspires.ftc.teamcode.HardwareControl;

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

    public Robot(){
        this.drivebase = drivebase;
        this.gyro = gyro;
    }

    public double getHeading(){
        return gyro.getHeading();
    }

    public void update(){
    }

}
