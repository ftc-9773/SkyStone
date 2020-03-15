package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.staticRegistrar;

import java.util.List;

public abstract class BaseOpMode extends LinearOpMode {
    private LynxModule.BulkCachingMode cachingMode = LynxModule.BulkCachingMode.MANUAL;
    List<LynxModule> revHubs;


    public abstract void initialise();

    public abstract void body();

    //Call once per cycle
    public void refreshCache(){
        for (LynxModule revHub: revHubs){
            revHub.clearBulkCache();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        revHubs = hardwareMap.getAll(LynxModule.class);
        List<LynxModule> revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule revHub: revHubs){
            revHub.setBulkCachingMode(cachingMode);
        }
        revHubs = hardwareMap.getAll(LynxModule.class);
        staticRegistrar.registerOpMode(this);
        initialise();
        waitForStart();
        body();

    }
}
