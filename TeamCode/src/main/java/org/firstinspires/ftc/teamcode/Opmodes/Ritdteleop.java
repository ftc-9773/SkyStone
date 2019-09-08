package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "WhyHelloThere")
public class Ritdteleop extends LinearOpMode {
    @Override
    public void runOpMode(){

        DcMotor leftIntakeMotor, rightIntakeMotor;
        Servo leftArmServo, rightArmServo;
        //leftIntakeMotor = hardwareMap.get(DcMotor.class, "LeftIntakeMotor");
        //rightIntakeMotor = hardwareMap.get(DcMotor.class, "RightIntakeMotor");

        leftArmServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightIntakeServo");

        waitForStart();

        while (opModeIsActive()){
//            if(gamepad1.a)
//            {
//                leftIntakeMotor.setPower(1);
//                rightIntakeMotor.setPower(-1);
//            } else{
//                leftIntakeMotor.setPower(0);
//                rightIntakeMotor.setPower(0);
//            }
            if (gamepad1.b){
                leftArmServo.setPosition(0.25);
                rightArmServo.setPosition(0.75);
            } else {
                leftArmServo.setPosition(0);
                rightArmServo.setPosition(1.00);
            }
        }
    }
}
