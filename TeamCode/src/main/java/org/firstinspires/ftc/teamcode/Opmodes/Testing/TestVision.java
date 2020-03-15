package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Logic.Vision.SkyStoneDetector;
import org.opencv.android.OpenCVLoader;

@TeleOp(name = "TestVision")
public class TestVision extends LinearOpMode {
    static {
        boolean magic = OpenCVLoader.initDebug();
        Log.d("ftc9773skyStoneDetector", "");
    }
    @Override
    public void runOpMode() throws InterruptedException {


        SkyStoneDetector detector = new SkyStoneDetector(true);
        detector.init(hardwareMap.appContext);
        detector.enable();
        sendTelemetry("Initliased");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.x){
                detector.refreshJson();
            }
            sendTelemetry("Reading " + detector.getPosition());
        }

    }

    public void sendTelemetry(String msg){
        telemetry.addLine(msg);
        telemetry.update();
    }
}
