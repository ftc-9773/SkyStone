package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

@TeleOp(name = "TestingTapeServo")
public class tapeServoTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo tapeServo = hardwareMap.get(Servo.class, "tapeServo");
        SafeJsonReader reader = new SafeJsonReader("RobotV1");
        double tapeZero = reader.getDouble("tapeZero");
        double tapeOut = reader.getDouble("tapeOut");
        long start = System.currentTimeMillis();
        waitForStart();
        while (opModeIsActive()){
            tapeServo.setPosition(tapeZero);
            telemetry.addLine("Set pos");
            telemetry.update();
            while (opModeIsActive() && start + 15000 > System.currentTimeMillis()){

            }
            tapeServo.setPosition(tapeOut);
            telemetry.addLine("Set pos again");
            telemetry.update();
            break;
        }
    }
}
