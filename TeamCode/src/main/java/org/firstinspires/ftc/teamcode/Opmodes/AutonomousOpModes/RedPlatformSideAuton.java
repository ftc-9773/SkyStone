package org.firstinspires.ftc.teamcode.Opmodes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedPlatformSideAuton")
public class RedPlatformSideAuton extends BasicRasiAuton {

    @Override
    public String fileName() {
        return "RedPlatformSideAuton";
    }

    @Override
    public boolean doVision() {
        return false;
    }
}
