package org.firstinspires.ftc.teamcode.Math;

import java.util.ArrayList;
import java.util.List;

public class Vector {
    public int len;
    public double mag;
    public double angle; //Only considers the first two numbers.
    ArrayList<Double> vector;


    public Vector(List<Double> v){
        vector = new ArrayList<>(v);
        len = vector.size();
        int sum = 0;
        for (double d:vector) {
            sum += d * d;
        }
        mag = Math.sqrt(sum);
        if (len >= 2){
            angle = Math.atan2(vector.get(1), vector.get(0));
        }
    }




}
