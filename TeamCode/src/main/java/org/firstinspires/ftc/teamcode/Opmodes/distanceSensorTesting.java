package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.DistSensor;

@TeleOp(name = "voltSensorTesting")
public class distanceSensorTesting extends LinearOpMode {

    @Override
    public void runOpMode(){

        DistSensor sensor;
        sensor = new DistSensor(hardwareMap, "distSensor0");


        waitForStart();
        while (opModeIsActive()){
            sendTelem("Distance: " + sensor.getReading());
        }
    }

    private void sendTelem(String msg){
        telemetry.addLine(msg);
        telemetry.update();
    }
}
