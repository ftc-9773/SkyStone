package org.firstinspires.ftc.teamcode.Utilities.Controllers;

public class VelocityController extends PIDController{
    double v_max;
    double a;
    double v; // Current velocity
    double v_hat;//Target velocity
    double p; //Assume maxV is at 100% power.
    double last_time;
    double min_power;
    double dt;
    double lastPos;
    double finalDist;
    /***
     * s (dist)
     * v(t) (target velocity at time t) (ticks/ms)
     * p(t) (power at time t)
     *
     * velocity increases at rate a(t)
     * a(t) = .1 * s * t  / 1 sec? //Preset this for all s based on some defined rate (4 inch / sec or whatever)
     *
     * after 1 second, v(t) = .1s per sec
     *
     * power(t) = v(t) + velocity PID correction
     *
     *
     * vPID = [kp, ki, kd]
     *
     * vPID(v_hat(t), v(t)):
     * 	e = v_hat(t) - v(t)
     * 	return kp * e + ki * int from 0 to t of e + kd * e'
     *
     * @param maxV Maximum velocity in units/ms
     * @param a Acceleration in units of maxV units / time
     * */
    public VelocityController(double maxV, double a, double min_power,  double kp, double kd, double ki){
        super(kp, kd, ki);
        this.v_max = maxV;
        this.a = a;
        this.v = 0;
        this.v_hat = 0;
        last_time = System.currentTimeMillis();
        this.min_power = min_power;

    }

    public double getPower(double targetPos, double actualPos){
        dt = System.currentTimeMillis() - last_time;
        last_time = System.currentTimeMillis();
        if (dt > 800) { ///If it's been a while since this was called, reset everything
            v = min_power * v_max;
            v_hat = v;
            lastPos = actualPos;
            p = min_power;
            finalDist = targetPos - actualPos; // For when you start, see how far you have to go.
        } else {
            v_hat += a(targetPos - actualPos) * dt;
            v = (actualPos - lastPos) / dt;
            p = getPIDCorrection(v_hat - v) + bound(-v_max, v_max, v_hat) / v_max; // Convert target velocity to power, adding PID correction to ensure conversion is accurate
            lastPos = actualPos;
        }
        return bound(-1, 1, p);

    }

    double a(double s){
        return a * (s < finalDist / 2 ? 1 : -1);
    }

    public double bound(double min, double max, double value){
        if (value < min){return min;}
        if (value > max){return max;}
        return value;
    }
}
