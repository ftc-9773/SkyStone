package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * For storing access to object instances that only need to be created once, so that we don't have to pass everything
 * in as an argument
 * */
public class staticRegistrar {
    public static LinearOpMode opMode = null; //Should be assigned at the beginning of each opmode

    public static void registerOpMode(LinearOpMode opMode){
        staticRegistrar.opMode = opMode;
    }
}
