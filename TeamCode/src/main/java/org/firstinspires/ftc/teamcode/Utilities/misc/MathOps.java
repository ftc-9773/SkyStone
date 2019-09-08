package org.firstinspires.ftc.teamcode.Utilities.misc;

public class MathOps {
    public static double modPi(double angle){
        return angle % Math.PI;
    }
    public static double modTwoPi (double angle) { return angle % (2 * Math.PI); }

    public static double setOnNegToPosPi (double num) {
        while (num > Math.PI) {
            num -= 2 * Math.PI;
        }
        while (num < -Math.PI) {
            num += 2 * Math.PI;
        }
        return num;
    }
    }
