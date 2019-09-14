package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;

public class Compass implements SensorEventListener {

    double lastReading;

    private SensorManager sensorManager;
    MagneticFlux flux; 
    public Compass(){
        ;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {

    }
}
