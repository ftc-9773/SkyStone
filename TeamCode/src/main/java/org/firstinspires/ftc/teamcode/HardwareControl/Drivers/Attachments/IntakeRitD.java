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

    public void intakeOn(){
        leftIntakeMotor.setPower(1);
        rightIntakeMotor.setPower(1);
    }

    @Override
    public boolean inStableState(){
        return false;
    }

    @Override
    public void stop(){}

    //Write Changes to device
    @Override
    public void update(){}
}
