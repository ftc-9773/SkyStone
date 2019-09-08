package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "distSensorTesting")
public class distanceSensorTesting extends LinearOpMode {

    @Override
    public void runOpMode(){

        DistanceSensor sensor;
        sensor = hardwareMap.get(DistanceSensor.class, "DistSensor");


        waitForStart();
        while (opModeIsActive()){
            sendTelem("Distance: " + sensor.getDistance(DistanceUnit.MM));
        }
    }

    private void sendTelem(String msg){
        telemetry.addLine(msg);
        telemetry.update();
    }
}
