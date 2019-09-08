package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class HorizontalLift implements Attachment {

    Servo leftClawServo, rightClawServo;
    SafeJsonReader jsonReader;

    double rightClawServoOpenPos, leftClasServoOpenPos;
    double rightClawServoClosePos, leftClawServoClosePos;
    double rightClawServoPos, leftClawServoPos;


    public HorizontalLift(HardwareMap hwmp) {
        leftClawServo = hwmp.get(Servo.class, "leftIntakeServo");
        rightClawServo = hwmp.get(Servo.class, "rightIntakeServo");

        jsonReader = new SafeJsonReader("Lifts");

        rightClawServoClosePos = jsonReader.getDouble("rClawServoClose");
        leftClawServoClosePos = jsonReader.getDouble("lClawServoClose");
        rightClawServoOpenPos = jsonReader.getDouble("rClawServoOpen");
        leftClasServoOpenPos = jsonReader.getDouble("lClawServoOpen");

    }

    public void grab(){
        rightClawServoPos = rightClawServoClosePos;
        leftClawServoPos = leftClawServoClosePos;
    }

    public void release(){
        rightClawServoPos = rightClawServoOpenPos;
        leftClawServoPos = leftClasServoOpenPos;
    }

    @Override
    public void stop() {
        release();
    }

    @Override
    public void update() {
        rightClawServo.setPosition(rightClawServoPos);
        leftClawServo.setPosition(leftClawServoPos);
    }
}
