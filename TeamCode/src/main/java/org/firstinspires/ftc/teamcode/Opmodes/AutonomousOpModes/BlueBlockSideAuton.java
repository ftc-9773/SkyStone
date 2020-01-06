package org.firstinspires.ftc.teamcode.Opmodes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueBlockSideAuton")
public class BlueBlockSideAuton extends BasicRasiAuton {
    @Override
    public String fileName() {
        return "BlueBlocksSideAuton";
    }

    @Override
    public boolean doVision() {
        return true;
    }
}
