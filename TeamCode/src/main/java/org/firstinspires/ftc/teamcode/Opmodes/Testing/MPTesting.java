package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import android.view.Display;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.Robot;
import org.firstinspires.ftc.teamcode.Logic.DriveUtil;

/**
 * Testing omnidirectional motion profile driving. Conclusion: It works
 * */

public class MPTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebase drivebase = new MecanumDrivebase(hardwareMap, telemetry);
        Gyro gyro = new Gyro(hardwareMap);
        Robot robot = new Robot(drivebase,gyro, hardwareMap);
        DriveUtil driveUtil = new DriveUtil(robot, this);

        waitForStart();
        driveUtil.drive(10, 45);
    }
}
