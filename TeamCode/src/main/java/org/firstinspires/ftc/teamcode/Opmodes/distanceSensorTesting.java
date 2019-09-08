package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "voltSensorTesting")
public class distanceSensorTesting extends LinearOpMode {

    @Override
    public void runOpMode(){

        VoltageSensor sensor;
        sensor = hardwareMap.voltageSensor.get("Motor Controller 1");


        waitForStart();
        while (opModeIsActive()){
            sendTelem("Distance: " + sensor.getVoltage());
        }
    }

    private void sendTelem(String msg){
        telemetry.addLine(msg);
        telemetry.update();
    }
}
