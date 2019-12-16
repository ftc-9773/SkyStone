package org.firstinspires.ftc.teamcode.Opmodes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BluePlatformSideAuton")
public class BluePlatformSideAuton extends BasicRasiAuton {
    @Override
    public String fileName() {
        return "BluePlatformSideAuton";
    }

    @Override
    public boolean doVision() {
        return false;
    }
}
