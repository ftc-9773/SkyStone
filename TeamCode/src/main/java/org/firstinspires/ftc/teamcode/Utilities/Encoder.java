package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.staticRegistrar;

public class Encoder {
    private DcMotor motor;

    public Encoder(){
        motor = staticRegistrar.opMode.hardwareMap.get(DcMotor.class, "[name]");
    }
}
