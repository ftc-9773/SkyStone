package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.Robot;
import org.firstinspires.ftc.teamcode.Logic.DriveUtil;

@Autonomous(name = "AlignementTest")
public class AlignmentTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebase drivebase = new MecanumDrivebase(hardwareMap, telemetry);
        Gyro gyro = new Gyro(hardwareMap);
        Robot robot = new Robot(drivebase, gyro, hardwareMap);
        DriveUtil driveUtil = new DriveUtil(robot, this);
        sendTelemetry("everything created");

        waitForStart();
        driveUtil.turnToAngle(45);
    }

    public void sendTelemetry(String msg){
        telemetry.addLine(msg);
        telemetry.update();
    }
}
