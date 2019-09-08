package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;


/**
 * @author  Cadence Weddle,
 * @version 1.0
 */

public class IntakeRitD implements Attachment {

    private DcMotor leftIntakeMotor, rightIntakeMotor;
    private Servo leftArmServo, rightArmServo;

    public double rightIntakeMotorPow, leftIntakeMotorPow;
    public double rightServoPos, leftServoPos;

    public double rightServoOpenPos, leftServoOpenPos;
    public double rightServoClosePos, leftServoClosePos;

    SafeJsonReader jsonReader;

    public IntakeRitD(HardwareMap hwmp){
        jsonReader = new SafeJsonReader("IntakeRitD");
        rightServoOpenPos = jsonReader.getDouble("rightServoOpenPos", 0);
        leftServoOpenPos = jsonReader.getDouble("leftServoOpenPos", 0);
        rightServoClosePos = jsonReader.getDouble("rightServoClosePos", 1);
        leftServoClosePos = jsonReader.getDouble("leftServoClosePos", 1);

        leftIntakeMotor = hwmp.get(DcMotor.class, "lIntakeM");
        rightIntakeMotor = hwmp.get(DcMotor.class, "rIntakeM");

        leftArmServo = hwmp.get(Servo.class, "lIntakeS");
        rightArmServo = hwmp.get(Servo.class, "rIntakeS");

        intakeClose();
        intakeOff();

        update();
    }

    public void intakeOpen(){
        rightServoPos = rightServoOpenPos;
        leftServoPos = leftServoOpenPos;
    }

    public void intakeClose(){
        rightServoPos = rightServoClosePos;
        leftServoPos = leftServoClosePos;
    }

    public void intakeOff(){
        leftIntakeMotorPow = 0;
        rightIntakeMotorPow = 0;
    }

    public void intakeOn(){
        leftIntakeMotorPow = 1;
        rightIntakeMotorPow = 1;
    }


    @Override
    public void stop(){
        intakeOff();
        intakeClose();
    }

    //Write Changes to device
    @Override
    public void update(){
        rightIntakeMotor.setPower(rightIntakeMotorPow);
        leftIntakeMotor.setPower(leftIntakeMotorPow);
        rightArmServo.setPosition(rightServoPos);
        leftArmServo.setPosition(leftServoPos);
    }
}
