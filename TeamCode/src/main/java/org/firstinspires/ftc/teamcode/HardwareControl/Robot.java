package org.firstinspires.ftc.teamcode.HardwareControl;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;

public class Robot {
    public MecanumDrivebase drivebase;
    public Gyro gyro;

    public Robot(MecanumDrivebase drivebase, Gyro gyro){
        this.drivebase = drivebase;
        this.gyro = gyro;
    }

    public void update(){
        this.gyro.isUpdated();
    }

}
