package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class BackHooks implements Attachment {
    private Servo leftHook, rightHook;
    double leftTargetPos, rightTargetPos;
    double leftDownPos, leftUpPos, rightDownPos, rightUpPos;
    private static final boolean DEBUG = false;

    SafeJsonReader reader;

    public BackHooks(HardwareMap hardwareMap){
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");

        reader = new SafeJsonReader("RobotV1");
        leftDownPos = reader.getDouble("leftBackHookDownPos");
        rightDownPos = reader.getDouble("rightBackHookDownPos");
        leftUpPos = reader.getDouble("leftBackHookUpPos");
        rightUpPos = reader.getDouble("rightBackHookUpPos");
        up();
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
        up();
        update();
    }

    @Override
    public void update() {
        if (DEBUG) Log.d("HOOKS","Set positions " + leftTargetPos + " " + rightHook);
        leftHook.setPosition(leftTargetPos);
        rightHook.setPosition(rightTargetPos);
    }
}
