package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;
import org.firstinspires.ftc.teamcode.Utilities.misc.Timer;

import java.sql.Time;

public class SideHook {
    Servo hook, arm;
    SafeJsonReader reader;
    double upPosA, downPosA, targetPosH, openPosH, cloasePosH, targetPosA, depositPosA;

    Timer downTimer = null;
    Timer upTimer = null;

    public SideHook(HardwareMap hardwareMap){
        hook = hardwareMap.get(Servo.class, "SideHook");
        arm = hardwareMap.get(Servo.class, "SideArm");
        reader = new SafeJsonReader("RobotV1");
        upPosA = reader.getDouble("sideArmUpPos");
        downPosA = reader.getDouble("sideArmDownPos");
        openPosH = reader.getDouble("sideHookOpenPos");
        cloasePosH = reader.getDouble("sideHookClosePos");
        depositPosA = reader.getDouble("sideArmDepositPos");
        targetPosH = cloasePosH;
        targetPosA = upPosA;
        update();
    }

    public void openHook(){
        if (targetPosA != upPosA) targetPosH = openPosH;
        hook.setPosition(targetPosH);
        //update();
    }

    public void down(){
        targetPosA = downPosA;
        arm.setPosition(targetPosA);
    }

    public void up(){
        targetPosA = upPosA;
        targetPosH = cloasePosH;
        arm.setPosition(targetPosA);
        hook.setPosition(targetPosH);
    }

    public void closeHook(){
        targetPosH = cloasePosH;
        hook.setPosition(targetPosH);
    }

    public void dropArm(){
        targetPosA = downPosA;
        targetPosH = openPosH;
        arm.setPosition(targetPosA);
        hook.setPosition(targetPosH);
    }


    public void update(){
        hook.setPosition(targetPosH);
        arm.setPosition(targetPosA);
    }

}
