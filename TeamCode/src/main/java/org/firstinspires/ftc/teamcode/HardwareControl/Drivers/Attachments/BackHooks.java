package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class BackHooks implements Attachment {
    private Servo leftHook, rightHook;
    double leftTargetPos, rightTargetPos;
    double leftDownPos, leftUpPos, rightDownPos, rightUpPos;

    SafeJsonReader reader;

    public BackHooks(HardwareMap hardwareMap){
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");

        reader = new SafeJsonReader("RobotV1");
        leftDownPos = reader.getDouble("leftBackHookDownPos");
        rightDownPos = reader.getDouble("rightBackHookDownPos");
        leftUpPos = reader.getDouble("leftBackHookUpPos");
        rightUpPos = reader.getDouble("rightBackHookUpPos");

    }

    public void down(){
        leftTargetPos = leftDownPos;
        rightTargetPos = rightDownPos;
    }

    public void up(){
        leftTargetPos = leftUpPos;
        rightTargetPos = rightUpPos;
    }

    @Override
    public void stop() {
        down();
        update();
    }

    @Override
    public void update() {
        leftHook.setPosition(leftTargetPos);
        rightHook.setPosition(rightTargetPos);
    }
}