package org.firstinspires.ftc.teamcode.Converters;

import org.firstinspires.ftc.teamcode.Math.Vector;

public abstract class BaseConverter extends Thread {
    static int inits = 0;
    public Vector state;

    //public abstract void start();


    public BaseConverter(){
        super("BaseConverter " + inits);
        inits += 1;
    }

    @Override
    public void run() {
        update();
    }

    public abstract void update();
}
