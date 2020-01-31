package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Intake;

@TeleOp(name = "distsensethingy")
public class TESTINGDISTSENSOR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addLine("dist: " + intake.touchSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
