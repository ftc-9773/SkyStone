package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.OrientationSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;


/**
 * Test IMU functions.
 * */
@Deprecated
public class testingstuff extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU imu;
        MagneticFlux flux;
        Orientation orientation;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                        = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        flux = imu.getMagneticFieldStrength();
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine("x:" + flux.x + " y:" + flux.y + " z:" + flux.z);
            telemetry.addLine("" + orientation.firstAngle + " " + orientation.secondAngle + " " + orientation.thirdAngle);
            telemetry.update();
            flux = imu.getMagneticFieldStrength();
            orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        }

    }
}
