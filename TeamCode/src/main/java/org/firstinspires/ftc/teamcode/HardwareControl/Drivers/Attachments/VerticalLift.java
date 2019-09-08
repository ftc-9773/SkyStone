package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class VerticalLift implements Attachment{

    SafeJsonReader jsonReader;
    DcMotor leftLiftMotor, rightLiftMotor;


    public VerticalLift(HardwareMap hwmp){


    }

    @Override
    public void stop() {

    }

    @Override
    public void update() {

    }
}
