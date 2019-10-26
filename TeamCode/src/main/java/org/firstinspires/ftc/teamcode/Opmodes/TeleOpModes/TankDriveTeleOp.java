package org.firstinspires.ftc.teamcode.Opmodes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.TankDrivebase;

@TeleOp(name = "TankDriveTeleOp")
public class TankDriveTeleOp extends LinearOpMode {

    @Override
    public void runOpMode(){
        TankDrivebase drivebase = new TankDrivebase(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            drivebase.driveHuman(gamepad1.left_stick_y, gamepad1.right_stick_y, false, true);
            drivebase.update();
        }
    }
}
