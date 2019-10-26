package org.firstinspires.ftc.teamcode.HardwareControl;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.DistSensor;

public class DistSensorArray {
    DistSensor rDistSensor, lDistSensor;

    public DistSensorArray(HardwareMap hardwareMap){
        rDistSensor = new DistSensor(hardwareMap, "rdistsensor");
        lDistSensor = new DistSensor(hardwareMap, "ldistsensor");
    }

    public double rReading(){
        return rDistSensor.getReading();
    }
    public double lReading() {
        return lDistSensor.getReading();
    }
    public double diff(){
        return rReading() - lReading();
    }
}
