package org.firstinspires.ftc.teamcode.Opmodes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Intake;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.RobotV1;


public class RobotV1TeleOpFull extends LinearOpMode {

    @Override
    public void runOpMode(){
        MecanumDrivebase drivebase = new MecanumDrivebase(hardwareMap, telemetry);
        sendTelemetry("Drivebase created");

        Intake intake = new Intake(hardwareMap);
        sendTelemetry("Intake created");

        Gyro gyro = new Gyro(hardwareMap);
        sendTelemetry("Gyro created");

        RobotV1 robot = new RobotV1(drivebase, gyro, intake);
        sendTelemetry("Robot created");


        sendTelemetry("Waiting for start...");
        robot.drivebase.runWithoutEncoders();

        // opmode start
        waitForStart();
        boolean temp = true;
        long lastTime = System.currentTimeMillis();
        while(opModeIsActive()) {
            if (temp){
                sendTelemetry("Started");
                temp = false;
            }

            robot.runGamepadCommands(gamepad1, gamepad2);
            robot.update();
            // interested in seeing cycle time
            long currTime = System.currentTimeMillis();
            telemetry.addData("elapsed time miliseconds", (currTime - lastTime));
            lastTime = currTime;

            telemetry.update();
        }

    }
    private void sendTelemetry(String msg){
        telemetry.addLine(msg);
        telemetry.update();
    }
}
