package org.firstinspires.ftc.teamcode.Utilities.Interrupters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class opModeInterrupter implements Interrupter {
    LinearOpMode opMode;

    public opModeInterrupter(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public boolean isNotInterrupted() {
        return !opMode.isStopRequested();
    }
}
