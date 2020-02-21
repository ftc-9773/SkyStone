package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class OdemetryWheelTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TouchSensor sensor;
        ColorSensor colorSensor;
        waitForStart();

        sensor = hardwareMap.get(TouchSensor.class, "TouchSensor");

        boolean readingTouch;
        long start;
        long next;
        long touchdt;
        while(opModeIsActive()){
            start = System.currentTimeMillis();
            readingTouch = sensor.isPressed();
            touchdt = System.currentTimeMillis() - start;
            start = start + touchdt;


            telemetry.addLine();
            telemetry.update();
        }
    }
}
