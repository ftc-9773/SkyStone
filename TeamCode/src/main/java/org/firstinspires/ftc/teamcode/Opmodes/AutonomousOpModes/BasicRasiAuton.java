package org.firstinspires.ftc.teamcode.Opmodes.AutonomousOpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.BackHooks;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Intake;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.RobotV1;
import org.firstinspires.ftc.teamcode.Logic.Vision.skyPositions;
import org.firstinspires.ftc.teamcode.RASI.Rasi.RasiInterpreter;
import org.firstinspires.ftc.teamcode.RASI.RasiCommands.RobotV1Commands;
import org.firstinspires.ftc.teamcode.Logic.Vision.SkyStoneDetector;

/**
 * implement filename to return the filename
 * ovverride doVision() to return false if you don't want to do vision
 * */
public abstract class BasicRasiAuton extends LinearOpMode {

    SkyStoneDetector detector;

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

        BackHooks backHooks = new BackHooks(hardwareMap);

        RobotV1 robot = new RobotV1(drivebase,gyro,intake, lifts, backHooks, telemetry);
        sendTelemetry("Robot created");

        //sendTelemetry("starting vision...");
        // wait to begin opMode
        RobotV1Commands rc = new RobotV1Commands(this, robot);
        RasiInterpreter rasiInterpreter = new RasiInterpreter("/sdcard/FIRST/team9773/Rasi2019/", fileName(), this, rc);

        if (doVision()) {

            // run vision
            detector = new SkyStoneDetector();
            // init the vision
            detector.init(hardwareMap.appContext);
            detector.enable();
            skyPositions blackPosition = skyPositions.center;
            skyPositions pos = skyPositions.unknown;
            int i = 0;
            while (!opModeIsActive() && !isStopRequested()) {
                //pos = detector.getPosition();
                if (pos != null) blackPosition = pos;
                telemetry.addData("VisionReading", blackPosition.toString());
                telemetry.update();
                i++;
            }
            // pass tags to RASI
            String[] tags = new String[1];
            tags[0] = Character.toString(blackPosition.toString().charAt(0)).toUpperCase();
            rasiInterpreter.setTags(tags);
            sendTelemetry("Set tags to " + tags[0]);
            Log.d("RasiAuto", "Set tag to " + tags[0]);

        }

        //rasiInterpreter.runRasiActually();
        sendTelemetry("Waiting for start");
        waitForStart();
        if (doVision()){
            detector.disable();
        }

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


