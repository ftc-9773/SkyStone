package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class SideHook {
    Servo hook;
    SafeJsonReader reader;
    double upPos, downPos, targetPos;


    public SideHook(HardwareMap hardwareMap){
        hook = hardwareMap.get(Servo.class, "SideHook");
        reader = new SafeJsonReader("RobotV1");
        upPos = reader.getDouble("sideHookUpPos");
        downPos = reader.getDouble("sideHookDownPos");
        targetPos = upPos;
    }

    public void up(){
        targetPos = upPos;
    }

    public void down(){
        targetPos = downPos;
    }

    public void update(){
        hook.setPosition(targetPos);
    }

}
