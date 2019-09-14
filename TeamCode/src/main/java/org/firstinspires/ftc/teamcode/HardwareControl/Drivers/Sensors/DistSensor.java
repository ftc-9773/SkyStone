package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistSensor {
    DistanceSensor sensor;
    double[] reading = new double[5];

    public DistSensor(HardwareMap hwmp, String name){
        this.sensor = hwmp.get(DistanceSensor.class, name);
        reading[0] = 0;
        reading[1] = 0;
        reading[2] = 0;
        reading[3] = 0;
        reading[4] = 0;
    }

    // Gets distance reading in centimeters
    public double getReading(){
        reading[0] = reading[1];
        reading[1] = reading[2];
        reading[2] = reading[3];
        reading[3] = reading[4];
        reading[4] = sensor.getDistance(DistanceUnit.MM);

        return (reading[0] + reading[1] + reading[2] + reading[3] + reading[4]) / 5;
    }


}
