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

    //DistSensorArray distSensorArray;

    long[] lastpos;


    //Default constructer
    public Robot(){}

    public Robot(MecanumDrivebase drivebase, Gyro gyro){
        this.drivebase = drivebase;
        this.gyro = gyro;
        //distSensorArray = new DistSensorArray(hardwareMap);
        lastpos = drivebase.getMotorPositions();
    }

    public double getHeading() {
        if (gyro != null) {
            return gyro.getHeading();
        }
        return -10000000; //Negative infinity
    }

    public void update(){
        drivebase.update();
        heading = getHeading();
        updateposition();
    }

    protected void updateposition(){
        long[] cur = drivebase.getMotorPositions();
        double dx = 0;
        dx += ((cur[0] - lastpos[0]) + (cur[1] - lastpos[1])) / 2;
        x += dx / MecanumDrivebase.COUNTS_PER_INCH;

        double dy = 0;
        dy += ((cur[0] - lastpos[0]) - (cur[1] - lastpos[1])) / 2;
        y += dy / MecanumDrivebase.COUNTS_PER_INCH;
        lastpos = cur;
    }

}
