package org.firstinspires.ftc.teamcode.Opmodes.AutonomousOpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "rasiTesting")
public class rasiTesting extends TestBasicRasiAuton {

    @Override
    public String fileName() {
        return "rasiTesting";
    }

    @Override
    public boolean doVision() {
        return false;
    }
}
