package org.firstinspires.ftc.teamcode.Logic;

import Jama.Matrix;

public class Controller {
    Matrix K;
    Matrix x, r, u;

    public Controller(Matrix K){
        this.K = K;
    }

    public Matrix getU(Matrix r, Matrix x){
        u = K.times(r.minus(x));
        return u;
    }

}
