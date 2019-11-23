package org.firstinspires.ftc.teamcode.Opmodes.AutonomousOpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.BackHooks;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Intake;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.RobotV1;
import org.firstinspires.ftc.teamcode.Logic.DriveUtil;
import org.firstinspires.ftc.teamcode.RASI.Rasi.RasiInterpreter;
import org.firstinspires.ftc.teamcode.RASI.RasiCommands.RasiCommands;
import org.firstinspires.ftc.teamcode.RASI.RasiCommands.RobotV1Commands;

/**
 * implement filename to return the filename
 * ovverride doVision() to return false if you don't want to do vision
 * */
public abstract class BasicRasiAuton extends LinearOpMode {

    public boolean doVision(){
        return true;
    }

    public void runOpMode(){
        // init robot.
        MecanumDrivebase drivebase = new MecanumDrivebase(hardwareMap, telemetry);
        sendTelemetry("Drivebase created");
        Intake intake = new Intake(hardwareMap);

        sendTelemetry("CubeLift created");
        Gyro gyro = new Gyro(hardwareMap);

        sendTelemetry("Gyro created");
        Lifts lifts = new Lifts(hardwareMap);
        sendTelemetry("Lifts created");

        sendTelemetry("Created back hooks interface");

        RobotV1 robot = new RobotV1(drivebase,gyro,intake, lifts, null, telemetry);
        sendTelemetry("Robot created");

        //sendTelemetry("starting vision...");
        // wait to begin opMode
        RobotV1Commands rc = new RobotV1Commands(this, robot);
        RasiInterpreter rasiInterpreter = new RasiInterpreter("/sdcard/FIRST/team9773/MercurialRasi/", fileName(), this, rc);

        //rasiInterpreter.runRasiActually();
        sendTelemetry("Waiting for start");
        waitForStart();

        // DO EVERYTHING
        rasiInterpreter.runRasiActually();
        //rasiInterpreter.run();

        robot.stop();
    }


    private void sendTelemetry(String msg){
        telemetry.addLine(msg);
        telemetry.update();
    }

    public abstract String fileName();

}


