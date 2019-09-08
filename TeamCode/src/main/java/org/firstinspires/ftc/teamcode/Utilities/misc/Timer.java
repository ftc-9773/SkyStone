package org.firstinspires.ftc.teamcode.Utilities.misc;

/**
 * @author Nicky
 * Timer to track how much time has elapsed.
 */
public class Timer {
    private long delayTimeMillis = 0;
    private long startTimeMillis = 0;

    /**
     * Creates Timer instance
     * @param delayInSeconds the amount of time in seconds before timer registers as done
     * */
    public Timer(double delayInSeconds) {
        delayTimeMillis = (long) (delayInSeconds * 1000.0);
        startTimeMillis = System.currentTimeMillis();
    }
    /**
     * Checks whether the amount of time specified in the creation of the Timer has elapsed
     * @return Boolean determining if above condition id true.
     * */
    public boolean isDone () {
        return System.currentTimeMillis() - startTimeMillis > delayTimeMillis;
    }
    /**
     * Checks whether the amount an amount of time has elapsed
     * @param delayInSeconds amount of time
     * @return boolean determining if input amount of time has elapsed
     * */
    public boolean isDone(double delayInSeconds) {
        return ((System.currentTimeMillis() - startTimeMillis) > delayInSeconds*1000);
    }
    /**
     * @return amount of time elapsed since Timer was initialised
     * */
    public double timeElapsedSeconds() {return (System.currentTimeMillis() - startTimeMillis) / 1000; }

    /**
     * Resets the start time to the current time
     * */
    public void restart(){
        this.startTimeMillis = System.currentTimeMillis();
    }
}
