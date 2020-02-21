package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Intake;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;

@TeleOp(name = "testingMagLimitSwitch")
public class TESTINGDISTSENSOR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Lifts intake = new Lifts(hardwareMap);
        intake.disablePIDLiftControl = true;
        waitForStart();
        while (opModeIsActive()){
            telemetry.addLine("dist: " +intake.readLimitSwitch());
            telemetry.update();
        }
    }
}
