package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;

public class testingstuff extends LinearOpMode {

    public MecanumDrivebase drivebase;
    public Gyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        drivebase = new MecanumDrivebase(hardwareMap, telemetry);
        gyro = new Gyro(hardwareMap);


    }
}
